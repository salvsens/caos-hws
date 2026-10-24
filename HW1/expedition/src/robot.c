// =============================================================================
// robot.c — автономный исследовательский робот (конечный автомат).
// Энергетическое правило: действие выполняется, только если после него
// энергии хватает до ближайшей зарядной станции (безопасной точки).
// =============================================================================
#include <string.h>
#include "model.h"

#define ID(r) ((int)((r) - w->rob))   // номер робота в массиве

const char* robot_state(RobotState s) {
    static const char* names[] = {"ожидание", "к цели", "исследование", "в зону связи", "передача",
                                  "к станции", "зарядка", "стоянка", "АВАРИЯ"};
    return names[s];
}

// Резерв: энергия на путь из p до ближайшей станции
int reserve(const World* w, Pos p) {
    int m = INF;
    for (int c = 0; c < w->nch; c++)
        if (w->dch[c][p.r][p.c] < m) m = w->dch[c][p.r][p.c];
    return m;
}

static int measure_energy(const World* w, int t) { return w->task[t].ticks * w->cfg.measure_cost; }

// Энергия на задание из клетки p: путь к цели + исследование + резерв от цели
static int task_need(const World* w, int t, Pos p) {
    int d = w->dtask[t][p.r][p.c];
    return d >= INF ? INF : d + measure_energy(w, t) + reserve(w, w->task[t].pos);
}

// Ближайшая работающая станция, до которой хватает энергии (-1 — нет)
static int best_charger(const World* w, const Robot* r) {
    int best = -1;
    for (int c = 0; c < w->nch; c++)
        if (w->chstock[c] != 0 && w->dch[c][r->pos.r][r->pos.c] <= r->energy &&
            (best < 0 || w->dch[c][r->pos.r][r->pos.c] < w->dch[best][r->pos.r][r->pos.c]))
            best = c;
    return best;
}

// Робот совместим с заданием и может его выполнить (сразу или после зарядки)
bool task_feasible(World* w, Robot* r, int t) {
    Task* k = &w->task[t];
    if (r->st == R_FAILED || (r->instr & k->instr) != k->instr || k->volume > r->mem_cap ||
        (k->unable & (1u << ID(r))))
        return false;
    if (task_need(w, t, r->pos) <= r->energy) return true;
    for (int c = 0; c < w->nch; c++)
        if (w->chstock[c] != 0 && w->dch[c][r->pos.r][r->pos.c] <= r->energy &&
            task_need(w, t, w->ch[c]) <= r->emax)
            return true;
    return false;
}

static int robot_at(const World* w, Pos p, int except) {
    for (int i = 0; i < w->nrob; i++)
        if (i != except && w->rob[i].st != R_FAILED && w->rob[i].pos.r == p.r && w->rob[i].pos.c == p.c)
            return i;
    return -1;
}

// Отказ от задания: диспетчер получает задание обратно, робот больше его не возьмёт
static void abandon(World* w, Robot* r, const char* why) {
    Task* t = &w->task[r->task];
    ev(w, 0, "ВНИМАНИЕ R%d отказывается от задания '%c': %s", ID(r) + 1, t->id, why);
    t->unable |= 1u << ID(r);
    if (t->robot == ID(r) && (t->st == T_ASSIGNED || t->st == T_ACTIVE)) t->st = T_PENDING;
    r->task = -1;
    r->measured = false;
    r->st = R_IDLE;
}

// Станция, после полной зарядки на которой задание t выполнимо (ближайшая по
// пути «робот -> станция -> цель»); -1 — такой нет
static int charger_for_task(const World* w, const Robot* r, int t) {
    int best = -1, best_d = INF;
    for (int c = 0; c < w->nch; c++) {
        int d = w->dch[c][r->pos.r][r->pos.c];
        if (w->chstock[c] != 0 && d <= r->energy && task_need(w, t, w->ch[c]) <= r->emax &&
            d + w->dtask[t][w->ch[c].r][w->ch[c].c] < best_d) {
            best = c;
            best_d = d + w->dtask[t][w->ch[c].r][w->ch[c].c];
        }
    }
    return best;
}

// Поездка на станцию c (зарядка до полной батареи); c < 0 — зарядиться негде: стоянка
static void go_charge(World* w, Robot* r, const char* why, int c) {
    if (c < 0) {
        if (r->task >= 0 && !r->measured) abandon(w, r, "негде пополнить энергию");
        if (r->st != R_PARKED) ev(w, 0, "ВНИМАНИЕ R%d встал на стоянку: %s, зарядиться негде", ID(r) + 1, why);
        r->st = R_PARKED;
        return;
    }
    r->goal = c;
    r->st = w->dch[c][r->pos.r][r->pos.c] == 0 ? R_CHARGE : R_TO_CHARGER;
    ev(w, 0, "МАРШРУТ  R%d: %s — к станции C%d (путь %d, энергия %d/%d)",
       ID(r) + 1, why, c + 1, w->dch[c][r->pos.r][r->pos.c], r->energy, r->emax);
}

// Выход в зону связи, если хватает энергии с резервом; иначе зарядка
static void go_comm(World* w, Robot* r, const char* why) {
    Pos e = field_end(w, w->dcomm, r->pos);
    int d = w->dcomm[r->pos.r][r->pos.c];
    if (d < INF && r->energy >= d + reserve(w, e)) {
        r->st = R_TO_COMM;
        ev(w, 0, "МАРШРУТ  R%d: %s — в зону связи (путь %d)", ID(r) + 1, why, d);
    } else {
        // с полной батареей заряжаться бессмысленно — стоянка
        go_charge(w, r, "не хватает энергии на выход в зону связи", r->energy < r->emax ? best_charger(w, r) : -1);
    }
}

// Выбор следующего действия
static void decide(World* w, Robot* r) {
    int i = ID(r);
    if (r->task >= 0 && !r->measured) {                       // есть задание без данных
        Task* t = &w->task[r->task];
        bool here = r->pos.r == t->pos.r && r->pos.c == t->pos.c;
        int need = here ? measure_energy(w, r->task) + reserve(w, r->pos) : task_need(w, r->task, r->pos);
        if (r->energy >= need) {
            if (here) {
                r->st = R_MEASURE;
                r->measure_left = t->ticks;
                ev(w, 0, "ИССЛЕД   R%d начал исследование цели '%c' (%d такт., %d пак.)", i + 1, t->id, t->ticks, t->volume);
            } else {
                r->st = R_TO_TARGET;
                ev(w, 0, "МАРШРУТ  R%d: маршрут к цели '%c' — стоимость %d; энергия %d >= нужно %d (путь+исследование+резерв)",
                   i + 1, t->id, w->dtask[r->task][r->pos.r][r->pos.c], r->energy, need);
            }
        } else if (charger_for_task(w, r, r->task) >= 0) {
            go_charge(w, r, "энергии на задание не хватает", charger_for_task(w, r, r->task));
        } else {
            abandon(w, r, "не хватает энергии даже после зарядки");
        }
        return;
    }
    if (r->nmem > 0) {                                        // есть данные для передачи
        if (!r->in_comm) go_comm(w, r, "связи нет, данные хранятся локально");
        else if (r->energy - w->cfg.tx_cost >= reserve(w, r->pos)) {
            if (r->st != R_UPLOAD) ev(w, 0, "ПЕРЕДАЧА R%d в зоне связи, в памяти %d пак.", i + 1, r->nmem);
            r->st = R_UPLOAD;
        } else go_charge(w, r, "не хватает энергии на передачу", best_charger(w, r));
        return;
    }
    if (r->task >= 0) {                                       // все данные подтверждены
        ev(w, 0, "ПОДТВ    R%d: все данные задания '%c' доставлены — робот свободен", i + 1, w->task[r->task].id);
        r->task = -1;
        r->measured = false;
    }
    if (r->energy * 100 < r->emax * w->cfg.recharge_pct && best_charger(w, r) >= 0) go_charge(w, r, "низкий заряд", best_charger(w, r));
    else if (!r->in_comm || w->grid[r->pos.r][r->pos.c] == '~') go_comm(w, r, "свободен, идёт ждать задание");
    else r->st = R_IDLE;
}

// Шаг в клетку n со списанием энергии
static void step(World* w, Robot* r, Pos n) {
    r->energy -= cell_cost(w, n);
    r->pos = n;
    r->moves++;
    r->blocked = 0;
    ev(w, 1, "ДВИЖЕНИЕ R%d -> (%d,%d)%s, энергия %d", ID(r) + 1, n.r, n.c,
       w->grid[n.r][n.c] == '~' ? " [опасная зона]" : "", r->energy);
    robot_comm_update(w, r);
}

// Уступить дорогу: шаг в случайную свободную соседнюю клетку с сохранением резерва
static void sidestep(World* w, Robot* r) {
    static const Pos D[4] = {{-1, 0}, {0, 1}, {1, 0}, {0, -1}};
    int k0 = rnd_int(4);
    for (int k = 0; k < 4; k++) {
        Pos n = {r->pos.r + D[(k0 + k) % 4].r, r->pos.c + D[(k0 + k) % 4].c};
        if (passable(w, n) && (robot_at(w, n, ID(r)) < 0 || is_dock(w, n)) &&
            r->energy - cell_cost(w, n) >= reserve(w, n)) {
            ev(w, 0, "ДВИЖЕНИЕ R%d уступает дорогу: отъезжает в (%d,%d)", ID(r) + 1, n.r, n.c);
            step(w, r, n);
            return;
        }
    }
}

// Движение по полю d. Возврат: true — робот у цели
static bool move_along(World* w, Robot* r, int d[MAXR][MAXC]) {
    if (d[r->pos.r][r->pos.c] == 0) return true;
    Pos n = step_to(w, d, r->pos);
    if (n.r == r->pos.r && n.c == r->pos.c) return false;   // пути нет
    int o = robot_at(w, n, ID(r));
    if (o >= 0 && !is_dock(w, n)) {                         // клетка занята — ждём
        ev(w, 1, "ДВИЖЕНИЕ R%d ждёт: клетка (%d,%d) занята R%d", ID(r) + 1, n.r, n.c, o + 1);
        w->rob[o].yield = true;
        if (++r->blocked >= 3) sidestep(w, r);              // затор: разъезд
        return false;
    }
    step(w, r, n);
    return d[r->pos.r][r->pos.c] == 0;
}

// Передача до 2 пакетов за такт: новые и неподтверждённые за ack_timeout
static void transmit(World* w, Robot* r) {
    int k = 0;
    for (int i = 0; i < r->nmem && k < 2 && r->in_comm; i++) {
        Packet* p = &r->mem[i];
        if (p->last_tx >= 0 && w->tick - p->last_tx < w->cfg.ack_timeout) continue;
        if (r->energy - w->cfg.tx_cost < reserve(w, r->pos)) break;
        bool retry = p->last_tx >= 0;
        r->energy -= w->cfg.tx_cost;
        p->last_tx = w->tick;
        r->sent++;
        r->retries += retry;
        ev(w, 0, "%s R%d -> база: пакет #%d (задание '%c', фрагмент %d), энергия %d",
           retry ? "ПОВТОР  " : "ПЕРЕДАЧА", ID(r) + 1, p->uid, w->task[p->task].id, p->seq + 1, r->energy);
        net_send(w, M_DATA, ID(r), p->task, p->uid, p->seq);
        k++;
    }
}

static void drop_packet(Robot* r, int i) {
    memmove(&r->mem[i], &r->mem[i + 1], (size_t)(r->nmem - i - 1) * sizeof r->mem[0]);
    r->nmem--;
}

// Вход в зону связи / выход из неё; при входе — сверка с базой:
// пакеты, уникальные номера которых уже есть на базе, считаются доставленными
void robot_comm_update(World* w, Robot* r) {
    bool now = r->st != R_FAILED && w->cover[r->pos.r][r->pos.c];
    if (now && !r->in_comm) {
        ev(w, 0, "СВЯЗЬ    R%d вошёл в зону связи (%d,%d)", ID(r) + 1, r->pos.r, r->pos.c);
        for (int i = 0; i < r->nmem;)
            if (w->got[r->mem[i].uid]) {
                ev(w, 0, "ПОДТВ    R%d: пакет #%d уже есть на базе (сверка по номеру) — доставлен", ID(r) + 1, r->mem[i].uid);
                drop_packet(r, i);
            } else i++;
    } else if (!now && r->in_comm && r->st != R_FAILED) {
        ev(w, 0, "СВЯЗЬ    R%d вышел из зоны связи%s", ID(r) + 1, r->nmem ? ", данные хранятся локально" : "");
    }
    r->in_comm = now;
}

void robot_on_assign(World* w, Robot* r, int t) {
    Task* k = &w->task[t];
    if (r->task == t) { ev(w, 0, "НАЗНАЧ   R%d: дубликат назначения '%c' проигнорирован", ID(r) + 1, k->id); return; }
    if (r->task >= 0 || (r->instr & k->instr) != k->instr || k->volume > r->mem_cap) {
        ev(w, 0, "НАЗНАЧ   R%d отклонил задание '%c' (занят или несовместимо)", ID(r) + 1, k->id);
        return;
    }
    r->task = t;
    r->measured = false;
    if (r->st != R_CHARGE) r->st = R_IDLE;
    ev(w, 0, "НАЗНАЧ   R%d получил задание '%c'", ID(r) + 1, k->id);
}

void robot_on_ack(World* w, Robot* r, int uid) {
    for (int i = 0; i < r->nmem; i++)
        if (r->mem[i].uid == uid) {
            drop_packet(r, i);
            ev(w, 0, "ПОДТВ    R%d получил подтверждение пакета #%d, в памяти %d", ID(r) + 1, uid, r->nmem);
            return;
        }
}

// Такт робота
void robot_tick(World* w, Robot* r) {
    int i = ID(r);
    if (r->st == R_FAILED) return;
    if (w->grid[r->pos.r][r->pos.c] == '~' && chance(w->cfg.danger_fail)) {   // авария в опасной зоне
        ev(w, 0, "АВАРИЯ   R%d вышел из строя в (%d,%d), потеряно пакетов: %d", i + 1, r->pos.r, r->pos.c, r->nmem);
        r->st = R_FAILED;
        r->in_comm = false;
        return;
    }
    // Сведения базы о задании (в зоне связи): выполнено другим или переназначено
    if (r->in_comm && r->task >= 0) {
        Task* t = &w->task[r->task];
        bool other = (t->st == T_ASSIGNED || t->st == T_ACTIVE) && t->robot != i && !r->measured;
        if (t->st == T_DONE || other) {
            ev(w, 0, "СВЯЗЬ    R%d: задание '%c' %s — робот свободен", i + 1, t->id,
               other ? "передано другому роботу" : r->measured ? "выполнено, все данные на базе" : "уже выполнено другим роботом");
            for (int k = 0; k < r->nmem;) {
                if (r->mem[k].task == r->task) drop_packet(r, k);
                else k++;
            }
            r->task = -1;
            r->measured = false;
            if (r->st != R_CHARGE && r->st != R_TO_CHARGER) r->st = R_IDLE;
        }
    }
    transmit(w, r);
    if (r->yield && (r->st == R_IDLE || r->st == R_UPLOAD) && !is_dock(w, r->pos)) sidestep(w, r);
    r->yield = false;
    switch (r->st) {
    case R_TO_TARGET:
        if (move_along(w, r, w->dtask[r->task])) {
            ev(w, 0, "МАРШРУТ  R%d прибыл к цели '%c', энергия %d", i + 1, w->task[r->task].id, r->energy);
            decide(w, r);
        }
        break;
    case R_TO_COMM:
        if (move_along(w, r, w->dcomm) || (r->in_comm && w->grid[r->pos.r][r->pos.c] != '~')) decide(w, r);
        break;
    case R_TO_CHARGER:
        if (move_along(w, r, w->dch[r->goal])) {
            r->st = R_CHARGE;
            ev(w, 0, "ЗАРЯДКА  R%d встал на зарядку на станции C%d, энергия %d/%d", i + 1, r->goal + 1, r->energy, r->emax);
        }
        break;
    case R_CHARGE:                       // энергию выдаёт станция (sim.c)
        if (r->energy >= r->emax || w->chstock[r->goal] == 0) {
            ev(w, 0, "ЗАРЯДКА  R%d покидает станцию C%d, энергия %d/%d", i + 1, r->goal + 1, r->energy, r->emax);
            r->st = R_IDLE;
            decide(w, r);
        }
        break;
    case R_MEASURE: {
        Task* t = &w->task[r->task];
        r->energy -= w->cfg.measure_cost;
        ev(w, 1, "ИССЛЕД   R%d исследует '%c', осталось %d, энергия %d", i + 1, t->id, r->measure_left - 1, r->energy);
        if (--r->measure_left == 0) {    // сохранение данных с уникальными номерами
            for (int s = 0; s < t->volume && w->next_uid < MAXPKT; s++)
                r->mem[r->nmem++] = (Packet){w->next_uid++, r->task, s, -1};
            r->measured = true;
            w->last_progress = w->tick;
            ev(w, 0, "ДАННЫЕ   R%d исследовал '%c': сохранено %d пак. (#%d..#%d)%s", i + 1, t->id, t->volume,
               w->next_uid - t->volume, w->next_uid - 1, r->in_comm ? "" : ", связи нет — хранит локально");
            r->st = R_IDLE;
            decide(w, r);
        }
        break;
    }
    default:                             // ожидание, передача, стоянка
        decide(w, r);
        break;
    }
}
