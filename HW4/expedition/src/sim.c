// =============================================================================
// sim.c — создание мира, ГСЧ, такт станции, инварианты и условия завершения.
// =============================================================================
#include <stdio.h>
#include "model.h"

// ------------------------------------------- ГСЧ xorshift64* (свой у каждого участника)
void rnd_seed(Rng* g, uint64_t seed, int id) {
    g->s = seed * 2654435761ULL + (uint64_t)(id + 1) * 0x9E3779B97F4A7C15ULL + 88172645463325252ULL;
    if (g->s == 0) g->s = 88172645463325252ULL;
}
static uint64_t rnd(Rng* g) {
    g->s ^= g->s >> 12; g->s ^= g->s << 25; g->s ^= g->s >> 27;
    return g->s * 2685821657736338717ULL;
}
int rnd_int(Rng* g, int n) { return n <= 1 ? 0 : (int)(rnd(g) % (uint64_t)n); }
bool chance(Rng* g, double p) { return p > 0 && (rnd(g) >> 11) * (1.0 / 9007199254740992.0) < p; }

// --------------------------------------------------------------- создание мира
int world_init(World* w, char* err, size_t n) {
    Config* c = &w->cfg;
    if (map_init(w, err, n) != 0) return -1;
    static Pos goals[MAXR * MAXC];
    // Задания: позиции целей берутся с карты
    w->ntask = c->ntask;
    for (int t = 0; t < c->ntask; t++) {
        Task* k = &w->task[t];
        k->id = c->tid[t]; k->instr = c->tinstr[t]; k->ticks = c->tticks[t]; k->volume = c->tvol[t];
        k->robot = -1; k->pos.r = -1;
        for (int r = 0; r < w->rows; r++)
            for (int q = 0; q < w->cols; q++)
                if (w->grid[r][q] == k->id) k->pos = (Pos){r, q};
        if (k->pos.r < 0) { snprintf(err, n, "цель '%c' описана, но её нет на карте", k->id); return -1; }
        field(w, &k->pos, 1, w->dtask[t]);
    }
    for (int r = 0; r < w->rows; r++)
        for (int q = 0; q < w->cols; q++) {
            char g = w->grid[r][q];
            bool described = false;
            for (int t = 0; t < w->ntask; t++) described |= w->task[t].id == g;
            if (g >= 'a' && g <= 'z' && !described) { snprintf(err, n, "у цели '%c' на карте нет описания target", g); return -1; }
        }
    for (int k = 0; k < w->nch; k++) field(w, &w->ch[k], 1, w->dch[k]);
    // Поле до зоны связи: ждать и передавать — не в опасных клетках
    int ng = 0;
    for (int r = 0; r < w->rows; r++)
        for (int q = 0; q < w->cols; q++)
            if (w->zone[r][q] >= 0 && w->grid[r][q] != '~') goals[ng++] = (Pos){r, q};
    field(w, goals, ng, w->dcomm);
    // Роботы стартуют на базе (стоянка для нескольких роботов) с полной батареей
    w->nrob = c->nrob;
    for (int i = 0; i < w->nrob; i++) {
        Robot* r = &w->rob[i];
        snprintf(r->name, sizeof r->name, "%s", c->rname[i]);
        r->instr = c->rinstr[i]; r->emax = r->energy = c->remax[i]; r->mem_cap = c->rmem[i];
        r->pos = w->base; r->task = -1; r->st = R_IDLE; r->in_comm = true;
        rnd_seed(&r->rng, c->seed, i);
        if (r->energy < reserve(w, r->pos)) { snprintf(err, n, "у робота %s не хватает энергии дойти до станции", r->name); return -1; }
    }
    for (int k = 0; k < w->nnode; k++) rnd_seed(&w->node[k].rng, c->seed, 100 + k);
    rnd_seed(&w->disp_rng, c->seed, 200);
    rnd_seed(&w->base_rng, c->seed, 201);
    w->next_uid = 1;
    dispatcher_init(w);
    return 0;
}

// Такт станции c: выдать энергию роботам на её разъёмах из запаса
void charger_tick(World* w, int c) {
    for (int i = 0; i < w->nrob; i++) {
        Robot* r = &w->rob[i];
        if (r->st != R_CHARGE || r->goal != c) continue;
        int give = w->cfg.charge_rate;
        if (give > r->emax - r->energy) give = r->emax - r->energy;
        if (w->chstock[c] >= 0 && give > w->chstock[c]) give = w->chstock[c];
        if (give <= 0) continue;
        r->energy += give; r->charged += give;
        if (w->chstock[c] > 0 && (w->chstock[c] -= give) == 0)
            ev(w, 0, "ВНИМАНИЕ станция C%d: запас энергии исчерпан", c + 1);
        ev(w, 1, "ЗАРЯДКА  C%d заряжает R%d: +%d -> %d/%d", c + 1, i + 1, give, r->energy, r->emax);
    }
}

// Проверка инвариантов модели после каждого такта
bool invariants(World* w) {
    w->checks++;
    for (int i = 0; i < w->nrob; i++) {
        Robot* r = &w->rob[i];
        if (r->energy < 0 || r->energy > r->emax)
            return snprintf(w->violation, sizeof w->violation, "энергия R%d вне [0, %d]: %d", i + 1, r->emax, r->energy), false;
        if (r->st != R_FAILED && r->energy < reserve(w, r->pos))
            return snprintf(w->violation, sizeof w->violation, "R%d не хватает энергии до станции", i + 1), false;
        if (r->task >= 0 && ((r->instr & w->task[r->task].instr) != w->task[r->task].instr ||
                             w->task[r->task].volume > r->mem_cap))
            return snprintf(w->violation, sizeof w->violation, "R%d выполняет несовместимое задание", i + 1), false;
        for (int j = i + 1; j < w->nrob; j++) {
            Robot* q = &w->rob[j];
            if (r->st != R_FAILED && q->st != R_FAILED && r->pos.r == q->pos.r && r->pos.c == q->pos.c && !is_dock(w, r->pos))
                return snprintf(w->violation, sizeof w->violation, "R%d и R%d в одной клетке", i + 1, j + 1), false;
        }
    }
    for (int c = 0; c < w->nch; c++) {                   // не больше роботов, чем разъёмов
        int busy = 0;
        for (int i = 0; i < w->nrob; i++) busy += w->rob[i].st == R_CHARGE && w->rob[i].goal == c;
        if (busy > w->cfg.charger_slots)
            return snprintf(w->violation, sizeof w->violation, "на станции C%d заряжаются %d роботов при %d разъёмах",
                            c + 1, busy, w->cfg.charger_slots), false;
    }
    int counted = 0;
    for (int u = 0; u < MAXPKT; u++) counted += w->got[u];
    if (counted != w->unique)
        return snprintf(w->violation, sizeof w->violation, "пакет засчитан базой дважды"), false;
    for (int t = 0; t < w->ntask; t++)
        if (w->task[t].st == T_DONE && w->task[t].nchunks != w->task[t].volume)
            return snprintf(w->violation, sizeof w->violation, "задание '%c' выполнено без всех данных", w->task[t].id), false;
    return true;
}

// Условия завершения
EndReason check_end(World* w) {
    if (w->end != END_NONE) return w->end;
    bool left = false, possible = false;
    for (int t = 0; t < w->ntask; t++) {
        if (w->task[t].st == T_DONE || w->task[t].st == T_UNAVAIL) continue;
        left = true;
        for (int i = 0; i < w->nrob; i++)                // есть исправный совместимый робот?
            if (w->rob[i].st != R_FAILED && (w->rob[i].instr & w->task[t].instr) == w->task[t].instr)
                possible = true;
    }
    if (!left) return w->end = END_DONE;
    if (!possible || w->tick - w->last_progress > w->cfg.stall_ticks) return w->end = END_EXHAUSTED;
    if (w->cfg.max_ticks > 0 && w->tick >= w->cfg.max_ticks) return w->end = END_TICKS;
    return END_NONE;
}
