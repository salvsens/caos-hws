// =============================================================================
// turn.c — снимок для шага робота, исполнение на клиенте, принятие на сервере.
// =============================================================================
#include <stdlib.h>
#include <string.h>
#include "protocol.h"

static int channel(const World* w, Pos p) {
    int node = w->zone[p.r][p.c];
    return node < 0 ? 0 : node;
}

void turn_make(const World* w, int id, Turn* t) {
    *t = (Turn){.tick = w->tick, .next_uid = w->next_uid};
    memcpy(t->rob, w->rob, sizeof t->rob);
    memcpy(t->stock, w->chstock, sizeof t->stock);
    for (int i = 0; i < w->ntask; i++) {
        t->task[i].st = w->task[i].st;
        t->task[i].robot = w->task[i].robot;
        t->task[i].unable = w->task[i].unable;
    }
    for (int i = 0; i < w->nnode; i++) t->queued[i] = w->node[i].nq;
    const Robot* r = &w->rob[id];
    for (int i = 0; i < r->nmem; i++) t->delivered[i] = w->got[r->mem[i].uid];
}

void turn_run(World* w, int id, const Turn* t, Step* s) {
    w->tick = t->tick;
    w->next_uid = t->next_uid;
    w->ev_head = 0;
    w->last_progress = -1;
    memcpy(w->rob, t->rob, sizeof w->rob);
    memcpy(w->chstock, t->stock, sizeof w->chstock);
    memset(w->got, 0, sizeof w->got);
    for (int i = 0; i < w->ntask; i++) {
        w->task[i].st = t->task[i].st;
        w->task[i].robot = t->task[i].robot;
        w->task[i].unable = t->task[i].unable;
    }

    Robot* r = &w->rob[id];
    for (int i = 0; i < r->nmem; i++) w->got[r->mem[i].uid] = t->delivered[i];
    int k = channel(w, r->pos);                  // передачи выполняются до движения
    w->node[k].nq = t->queued[k];
    robot_tick(w, r);

    *s = (Step){.rob = *r, .tick = t->tick, .next_uid = w->next_uid,
                .progress = w->last_progress == w->tick, .abandoned = -1};
    for (int i = 0; i < w->nrob; i++)
        if (i != id && w->rob[i].yield && !t->rob[i].yield) s->yielded |= 1u << i;
    for (int i = 0; i < w->ntask; i++)
        if (w->task[i].unable != t->task[i].unable) s->abandoned = i;
    s->ntx = w->node[k].nq - t->queued[k];
    for (int i = 0; i < s->ntx; i++) {
        Msg* m = &w->node[k].q[t->queued[k] + i];
        s->tx[i].uid = m->uid;
        s->tx[i].at = m->at;
    }
    s->nev = (int)(w->ev_head < STEP_EVENTS ? w->ev_head : STEP_EVENTS);
    memcpy(s->events, w->evq, (size_t)s->nev * sizeof s->events[0]);
}

static const Packet* find_packet(const Robot* r, int uid) {
    for (int i = 0; i < r->nmem; i++)
        if (r->mem[i].uid == uid) return &r->mem[i];
    return NULL;
}

// Проверки выполняются до любых изменений мира.
bool turn_apply(World* w, int id, const Step* s) {
    const Robot* old = &w->rob[id];
    const Robot* r = &s->rob;
    if (s->tick != w->tick || s->next_uid < w->next_uid || s->next_uid > MAXPKT ||
        s->next_uid - w->next_uid > MAXMEM || (unsigned)s->progress > 1 ||
        s->abandoned < -1 || (s->abandoned >= 0 && s->abandoned != old->task) ||
        s->yielded >= 1u << w->nrob || (s->yielded & (1u << id)) ||
        (unsigned)s->ntx > STEP_TX || (unsigned)s->nev > STEP_EVENTS) return false;
    if (!passable(w, r->pos) || abs(r->pos.r - old->pos.r) + abs(r->pos.c - old->pos.c) > 1 ||
        (unsigned)r->st > R_FAILED || r->energy < 0 || r->energy > old->emax ||
        r->instr != old->instr || r->emax != old->emax || r->mem_cap != old->mem_cap ||
        memcmp(r->name, old->name, sizeof r->name) || r->charged != old->charged ||
        r->task < -1 || (r->task >= 0 && r->task != old->task) ||
        r->nmem < 0 || r->nmem > r->mem_cap || r->measured > 1 || r->in_comm > 1 || r->yield > 1)
        return false;
    if ((r->st == R_TO_TARGET || r->st == R_MEASURE) && r->task < 0) return false;
    if (r->st == R_MEASURE && (r->measure_left < 1 || r->measure_left > w->task[r->task].ticks)) return false;
    if ((r->st == R_QUEUE || r->st == R_TO_CHARGER || r->st == R_CHARGE) && (unsigned)r->goal >= (unsigned)w->nch)
        return false;
    if (r->sent < old->sent || r->sent - old->sent > STEP_TX ||
        s->ntx > r->sent - old->sent || r->moves < old->moves || r->moves - old->moves > 1 ||
        r->retries < old->retries || r->retries - old->retries > STEP_TX ||
        r->blocked < 0 || r->blocked > w->tick) return false;
    for (int i = 0; i < r->nmem; i++) {
        const Packet* p = &r->mem[i];
        if (p->uid < 1 || p->uid >= s->next_uid || (unsigned)p->task >= (unsigned)w->ntask ||
            (unsigned)p->seq >= (unsigned)w->task[p->task].volume || p->last_tx < -1 || p->last_tx > w->tick ||
            find_packet(r, p->uid) != p) return false;
    }
    Node* node = &w->node[channel(w, old->pos)];
    if (s->ntx > NODEQ - node->nq) return false;
    Msg out[STEP_TX];
    for (int i = 0; i < s->ntx; i++) {
        const Packet* p = find_packet(old, s->tx[i].uid);
        int at = s->tx[i].at, earliest = w->tick + w->cfg.net_delay;
        if (!p || at < earliest || at > earliest + w->cfg.net_jitter) return false;
        out[i] = (Msg){M_DATA, id, p->task, p->uid, p->seq, at, false};
    }
    for (int i = 0; i < s->nev; i++)
        if ((unsigned)s->events[i].level > 1 || !memchr(s->events[i].text, 0, EVLEN)) return false;

    w->sent[M_DATA] += r->sent - old->sent;
    w->lost[M_DATA] += r->sent - old->sent - s->ntx;
    for (int i = 0; i < s->ntx; i++) node->q[node->nq++] = out[i];
    w->rob[id] = *r;
    w->next_uid = s->next_uid;
    for (int i = 0; i < w->nrob; i++)
        if (s->yielded & (1u << i)) w->rob[i].yield = true;
    if (s->abandoned >= 0) {
        Task* t = &w->task[s->abandoned];
        t->unable |= 1u << id;
        if (t->robot == id && (t->st == T_ASSIGNED || t->st == T_ACTIVE)) t->st = T_PENDING;
    }
    if (s->progress) w->last_progress = w->tick;
    for (int i = 0; i < s->nev; i++) w->evq[w->ev_head++ % EVQ] = s->events[i];
    return true;
}
