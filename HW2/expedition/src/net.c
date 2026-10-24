// =============================================================================
// net.c — связь и управление: узлы связи (антенна базы и ретрансляторы
// пересылают сообщения с задержкой, потерей и дублированием), поток базы
// (приём пакетов по уникальным номерам, подтверждения) и диспетчер
// (назначение заданий, тайм-ауты, повторное назначение).
// Все функции, кроме base_thread, вызываются с захваченным мьютексом мира.
// =============================================================================
#include <stdio.h>
#include <string.h>
#include "model.h"

static const char* MSG_NAME[3] = {"назначение", "пакет", "подтверждение"};

// Описание сообщения: назначение — буквой задания, пакет и подтверждение — номером
static const char* msg_text(const World* w, const Msg* m, char* buf, size_t n) {
    if (m->type == M_ASSIGN) snprintf(buf, n, "назначение '%c' для R%d", w->task[m->task].id, m->robot + 1);
    else snprintf(buf, n, "%s #%d (R%d)", MSG_NAME[m->type], m->uid, m->robot + 1);
    return buf;
}

// ---------------------------------------------------------------- канал связи
// Отправка: сообщение встаёт в очередь узла, в зоне которого находится робот
void net_send(World* w, Rng* g, MsgType t, int robot, int task, int uid, int seq) {
    Pos p = w->rob[robot].pos;
    int k = w->zone[p.r][p.c] >= 0 ? w->zone[p.r][p.c] : 0;
    Node* nd = &w->node[k];
    w->sent[t]++;
    if (nd->nq == NODEQ) {
        w->lost[t]++;
        ev(w, 0, "СЕТЬ     очередь узла %d переполнена — сообщение потеряно", k);
        return;
    }
    nd->q[nd->nq++] = (Msg){t, robot, task, uid, seq, w->tick + w->cfg.net_delay + rnd_int(g, w->cfg.net_jitter + 1), false};
}

// Такт узла связи: пересылка сообщений, срок доставки которых наступил
void node_tick(World* w, int k) {
    const Config* c = &w->cfg;
    Node* nd = &w->node[k];
    char nm[40], txt[80];
    snprintf(nm, sizeof nm, k ? "ретранслятор ^%d" : "антенна B", k);
    Msg due[NODEQ];
    int ndue = 0, keep = 0;
    for (int i = 0; i < nd->nq; i++) {
        if (nd->q[i].at <= w->tick) due[ndue++] = nd->q[i];
        else nd->q[keep++] = nd->q[i];
    }
    nd->nq = keep;
    for (int i = 0; i < ndue; i++) {
        Msg* m = &due[i];
        Robot* r = &w->rob[m->robot];
        msg_text(w, m, txt, sizeof txt);
        if (chance(&nd->rng, c->p_loss)) {
            nd->lost++;
            w->lost[m->type]++;
            ev(w, 0, "СЕТЬ     %s: потеряно в канале — %s", nm, txt);
            continue;
        }
        if (m->type == M_DATA) {                  // пакет — в ящик базы, база проснётся
            if (w->in_count < MBOX) {
                w->inbox[(w->in_head + w->in_count++) % MBOX] = *m;
                sem_post(w->inbox_sem);
                ev(w, 1, "СЕТЬ     %s: передано на базу — %s", nm, txt);
            } else ev(w, 0, "СЕТЬ     %s: ящик базы переполнен, потеряно — %s", nm, txt);
        } else if (r->st == R_FAILED || !r->in_comm) {
            ev(w, 0, "СЕТЬ     %s: не доставлено (робот вне зоны связи) — %s", nm, txt);
        } else if (m->type == M_ASSIGN) {
            robot_on_assign(w, r, m->task);
        } else {
            robot_on_ack(w, r, m->uid);
        }
        nd->fwd++;
        if (!m->copy && nd->nq < NODEQ && chance(&nd->rng, c->p_dup)) {   // копия придёт ещё раз
            Msg copy = *m;
            copy.copy = true;
            copy.at = w->tick + 1 + rnd_int(&nd->rng, c->net_jitter + 1);
            nd->q[nd->nq++] = copy;
            nd->dup++;
            w->dupd[m->type]++;
            ev(w, 0, "СЕТЬ     %s: продублировано — %s", nm, txt);
        }
    }
}

// --------------------------------------------------------------------- база
// Каждый уникальный номер засчитывается один раз, на каждый пакет — подтверждение
static void base_receive(World* w, const Msg* m) {
    Task* t = &w->task[m->task];
    if (w->got[m->uid]) {
        w->dups++;
        ev(w, 0, "БАЗА     пакет #%d задания '%c' от R%d — дубликат, уже засчитан, отброшен", m->uid, t->id, m->robot + 1);
    } else {
        w->got[m->uid] = true;
        w->unique++;
        w->last_progress = w->tick;
        if (!t->chunk[m->seq]) { t->chunk[m->seq] = true; t->nchunks++; }
        ev(w, 0, "БАЗА     принят пакет #%d от R%d: задание '%c', фрагмент %d/%d (собрано %d/%d)",
           m->uid, m->robot + 1, t->id, m->seq + 1, t->volume, t->nchunks, t->volume);
        if (t->nchunks == t->volume && t->st != T_DONE) {
            t->st = T_DONE;
            t->done_tick = w->tick;
            ev(w, 0, "ВЫПОЛНЕНО задание '%c': база получила все %d фрагмента данных", t->id, t->volume);
        }
    }
    net_send(w, &w->base_rng, M_ACK, m->robot, m->task, m->uid, m->seq);
}

// Поток базы: не привязан к тактам — спит на семафоре, пока в ящике нет пакетов
void* base_thread(void* arg) {
    World* w = arg;
    for (;;) {
        while (sem_wait(w->inbox_sem) != 0) {}    // прерывание сигналом — ждать дальше
        pthread_mutex_lock(&w->mx);
        Msg m = w->inbox[w->in_head];
        w->in_head = (w->in_head + 1) % MBOX;
        w->in_count--;
        if (m.type != M_STOP) base_receive(w, &m);
        pthread_mutex_unlock(&w->mx);
        if (m.type == M_STOP) return NULL;
    }
}

// ------------------------------------------------------------------ диспетчер
// Начальная проверка: может ли хоть один робот выполнить задание
void dispatcher_init(World* w) {
    for (int t = 0; t < w->ntask; t++) {
        Task* k = &w->task[t];
        bool instr = false, feasible = false;
        for (int i = 0; i < w->nrob; i++) {
            if ((w->rob[i].instr & k->instr) == k->instr) instr = true;
            if (task_feasible(w, &w->rob[i], t)) feasible = true;
        }
        if (feasible) continue;
        char ins[64];
        format_instr(k->instr, ins, sizeof ins);
        if (!instr) snprintf(k->note, sizeof k->note, "нет робота с приборами %s", ins);
        else if (w->dtask[t][w->base.r][w->base.c] >= INF) snprintf(k->note, sizeof k->note, "цель недостижима");
        else snprintf(k->note, sizeof k->note, "не хватает памяти или ёмкости батарей роботов");
        k->st = T_UNAVAIL;
        ev(w, 0, "ВНИМАНИЕ задание '%c' недоступно: %s", k->id, k->note);
    }
}

static void assign(World* w, int ri, int ti) {
    Robot* r = &w->rob[ri];
    Task* t = &w->task[ti];
    t->st = T_ASSIGNED;
    t->robot = ri;
    t->t_assign = w->tick;
    t->attempts++;
    // допустимое молчание исполнителя: путь туда и обратно + исследование + запас
    t->limit = 2 * w->dtask[ti][r->pos.r][r->pos.c] + t->ticks + w->cfg.task_timeout;
    ev(w, 0, "НАЗНАЧ   диспетчер -> R%d: задание '%c'%s, путь %d, энергия %d/%d",
       ri + 1, t->id, t->attempts > 1 ? " (повторно)" : "", w->dtask[ti][r->pos.r][r->pos.c], r->energy, r->emax);
    net_send(w, &w->disp_rng, M_ASSIGN, ri, ti, 0, 0);
}

// Такт диспетчера
void dispatcher_tick(World* w) {
    // 1. Телеметрия от роботов в зоне связи: подтверждение и контакт
    for (int i = 0; i < w->nrob; i++) {
        Robot* r = &w->rob[i];
        if (r->st == R_FAILED || !r->in_comm || r->task < 0) continue;
        Task* t = &w->task[r->task];
        if (t->st == T_ASSIGNED && t->robot == i) {
            t->st = T_ACTIVE;
            ev(w, 0, "НАЗНАЧ   R%d подтвердил получение задания '%c'", i + 1, t->id);
        }
        if (t->st == T_PENDING) { t->st = T_ACTIVE; t->robot = i; }   // связь с «потерянным» исполнителем восстановлена
        if (t->st == T_ACTIVE && t->robot == i) t->last_contact = w->tick;
    }
    // 2. Тайм-ауты: неподтверждённое назначение и молчащий исполнитель
    for (int k = 0; k < w->ntask; k++) {
        Task* t = &w->task[k];
        if (t->st == T_ASSIGNED && w->tick - t->t_assign > w->cfg.assign_timeout) {
            ev(w, 0, "ТАЙМАУТ  R%d не подтвердил задание '%c' — назначение потеряно", t->robot + 1, t->id);
            t->st = T_PENDING;
        } else if (t->st == T_ACTIVE && w->tick - t->last_contact > t->limit) {
            ev(w, 0, "ТАЙМАУТ  задание '%c' потеряно: нет связи с R%d %d тактов — будет назначено повторно",
               t->id, t->robot + 1, w->tick - t->last_contact);
            t->st = T_PENDING;
        }
    }
    // 3. Назначение: свободным роботам в зоне связи; задание, уже назначенное
    //    кому-то, повторно не назначается (нет необоснованного дублирования)
    for (int i = 0; i < w->nrob; i++) {
        Robot* r = &w->rob[i];
        if (r->st == R_FAILED || !r->in_comm || r->task >= 0 || r->nmem > 0) continue;
        bool waiting = false;   // робот уже ждёт назначение
        for (int k = 0; k < w->ntask; k++)
            if (w->task[k].st == T_ASSIGNED && w->task[k].robot == i) waiting = true;
        if (waiting) continue;
        int best = -1;
        for (int k = 0; k < w->ntask; k++) {
            if (w->task[k].st != T_PENDING || !task_feasible(w, r, k)) continue;
            if (best < 0 || (w->cfg.strategy == 1 &&
                             w->dtask[k][r->pos.r][r->pos.c] < w->dtask[best][r->pos.r][r->pos.c]))
                best = k;
            if (w->cfg.strategy == 0) break;   // fifo — первое подходящее по порядку
        }
        if (best >= 0) assign(w, i, best);
    }
}
