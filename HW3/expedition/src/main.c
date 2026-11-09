// =============================================================================
// main.c — главная программа и процесс-«часы»: создание мира в разделяемой
// памяти, порождение процессов-участников, события (экран + журнал через
// дескрипторы), отображение карты, обработка сигналов, итоговый отчёт.
// =============================================================================
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "model.h"

static World g_init;                         // мир готовится здесь и копируется в разделяемую память
static World* volatile g_w = NULL;           // мир в разделяемой памяти (для обработчика сигналов)
static int g_log = -1;                       // дескриптор файла журнала (только у часов)
static unsigned g_shown = 0;                 // сколько событий из w->evq часы уже вывели
static bool g_echo = false;                  // независимый участник: свои события — в свой терминал
static char g_recent[10][EVLEN];             // последние события для режима карты
static int g_nrecent = 0;

// Событие (в любом процессе, под захватом мира): строка в кольцевой буфер мира.
// Выводят события часы — flush_events.
void ev(World* w, int level, const char* fmt, ...) {
    char text[EVLEN - 16];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(text, sizeof text, fmt, ap);
    va_end(ap);
    unsigned k = w->ev_head++ % EVQ;
    w->evq[k].level = level;
    snprintf(w->evq[k].text, EVLEN, "[%04d] %s", w->tick % 10000, text);
    if (g_echo && level <= w->cfg.verbose) dprintf(1, "%s\n", w->evq[k].text);
}

// Часы (под захватом мира): новые события — в журнал всегда, на экран — уровень
// не выше verbose (в режиме карты — в список последних)
static void flush_events(World* w) {
    if (w->ev_head - g_shown > EVQ) {
        if (g_log >= 0) dprintf(g_log, "... пропущено событий: %u\n", w->ev_head - g_shown - EVQ);
        g_shown = w->ev_head - EVQ;
    }
    for (; g_shown != w->ev_head; g_shown++) {
        const char* text = w->evq[g_shown % EVQ].text;
        if (g_log >= 0) dprintf(g_log, "%s\n", text);
        if (w->evq[g_shown % EVQ].level > w->cfg.verbose) continue;
        if (w->cfg.map_view) {
            memmove(g_recent[1], g_recent[0], sizeof g_recent - sizeof g_recent[0]);
            snprintf(g_recent[0], sizeof g_recent[0], "%s", text);
            if (g_nrecent < 10) g_nrecent++;
        } else {
            dprintf(1, "%s\n", text);
        }
    }
}

// Обработчик SIGINT/SIGTERM в любом процессе: отметить сигнал в разделяемой памяти
static void on_signal(int sig) {
    if (g_w) g_w->signo = sig;
}

// Кадр режима карты (под захватом мира): карта с роботами, роботы, станции, последние события
static void draw(World* w) {
    dprintf(1, "\033[H\033[2J== Планетарная экспедиция (процессов: %d + часы) == такт %d (лимит %d), seed %llu\n",
            w->npart + 1, w->tick, w->cfg.max_ticks, (unsigned long long)w->cfg.seed);
    for (int r = 0; r < w->rows; r++) {
        char line[MAXC + 1];
        for (int c = 0; c < w->cols; c++) {
            char g = w->grid[r][c];
            for (int t = 0; t < w->ntask; t++)          // выполненная цель — заглавной буквой
                if (w->task[t].st == T_DONE && w->task[t].pos.r == r && w->task[t].pos.c == c) g = w->task[t].id - 32;
            for (int i = w->nrob - 1; i >= 0; i--)
                if (w->rob[i].pos.r == r && w->rob[i].pos.c == c) g = w->rob[i].st == R_FAILED ? 'X' : (char)('1' + i);
            line[c] = g;
        }
        line[w->cols] = '\0';
        dprintf(1, "  %s\n", line);
    }
    dprintf(1, "  # скалы ~ опасно B база C станция ^ ретранслятор a-z цели (A-Z выполнены) 1-9 роботы X авария\n");
    for (int i = 0; i < w->nrob; i++) {
        Robot* r = &w->rob[i];
        dprintf(1, "  R%d %-8s энергия %3d/%-3d память %d/%d задание %c  %s, %s\n", i + 1, r->name, r->energy,
                r->emax, r->nmem, r->mem_cap, r->task >= 0 ? w->task[r->task].id : '-',
                r->in_comm ? "связь" : "нет связи", robot_state(r->st));
    }
    int done = 0, transit = 0;
    for (int t = 0; t < w->ntask; t++) done += w->task[t].st == T_DONE;
    for (int k = 0; k < w->nnode; k++) transit += w->node[k].nq;
    dprintf(1, "  Задания: выполнено %d из %d | база: пакетов %d, дубликатов %d | в пути %d\n",
            done, w->ntask, w->unique, w->dups, transit);
    for (int i = g_nrecent - 1; i >= 0; i--) dprintf(1, "  %s\n", g_recent[i]);
}

// Итоговый отчёт: причина, задания, недоставленные данные, статистика
static void report(World* w, int fd, int code) {
    static const char* why[] = {"", "выполнены все доступные задания", "достигнут предел тактов",
                                "ресурсы экспедиции исчерпаны", "прервано пользователем", "НАРУШЕН ИНВАРИАНТ"};
    static const char* st[] = {"ожидает", "назначено", "выполняется", "ВЫПОЛНЕНО", "недоступно"};
    dprintf(fd, "\n===== ИТОГИ ЭКСПЕДИЦИИ =====\nПричина завершения: %s %s\nТактов: %d, seed: %llu, процессов: %d + часы\n",
            why[w->end], w->end == END_INVARIANT ? w->violation : "", w->tick, (unsigned long long)w->cfg.seed, w->npart + 1);
    dprintf(fd, "Задания:\n");
    for (int t = 0; t < w->ntask; t++) {
        Task* k = &w->task[t];
        char ins[64];
        format_instr(k->instr, ins, sizeof ins);
        dprintf(fd, "  '%c' %-10s %-11s данные %d/%d, назначений %d", k->id, ins, st[k->st], k->nchunks, k->volume, k->attempts);
        if (k->st == T_DONE) dprintf(fd, ", такт %d\n", k->done_tick);
        else dprintf(fd, "%s%s\n", k->note[0] ? " — " : "", k->note);
    }
    dprintf(fd, "Невыполненные задания:");
    int un = 0;
    for (int t = 0; t < w->ntask; t++)
        if (w->task[t].st != T_DONE) { dprintf(fd, " '%c'", w->task[t].id); un++; }
    dprintf(fd, "%s\nНедоставленные данные:", un ? "" : " нет");
    int nd = 0;
    for (int i = 0; i < w->nrob; i++)
        for (int k = 0; k < w->rob[i].nmem; k++)
            if (!w->got[w->rob[i].mem[k].uid]) {
                dprintf(fd, " #%d('%c', у R%d)", w->rob[i].mem[k].uid, w->task[w->rob[i].mem[k].task].id, i + 1);
                nd++;
            }
    dprintf(fd, "%s\nРоботы:\n", nd ? "" : " нет");
    for (int i = 0; i < w->nrob; i++) {
        Robot* r = &w->rob[i];
        dprintf(fd, "  R%d %-8s энергия %3d/%-3d шагов %3d, передач %2d (повторов %2d), получено энергии %3d, %s\n",
                i + 1, r->name, r->energy, r->emax, r->moves, r->sent, r->retries, r->charged, robot_state(r->st));
    }
    dprintf(fd, "Узлы связи (переслано/потеряно/продублировано):");
    for (int k = 0; k < w->nnode; k++)
        dprintf(fd, " %s%.0d %d/%d/%d;", k ? "^" : "B", k, w->node[k].fwd, w->node[k].lost, w->node[k].dup);
    dprintf(fd, "\n");
    dprintf(fd, "Связь: отправлено назначений/пакетов/подтверждений %d/%d/%d, потеряно %d/%d/%d, продублировано %d/%d/%d\n",
            w->sent[0], w->sent[1], w->sent[2], w->lost[0], w->lost[1], w->lost[2], w->dupd[0], w->dupd[1], w->dupd[2]);
    dprintf(fd, "База: засчитано уникальных пакетов %d, отброшено дубликатов %d\n", w->unique, w->dups);
    dprintf(fd, "Инварианты: проверок %d, %s\nКод завершения: %d\n", w->checks,
            w->end == END_INVARIANT ? "есть нарушение" : "нарушений нет", code);
}

// Ctrl+C, SIGTERM: обработчик только отмечает сигнал. Без SA_RESTART — сигнал
// прерывает nanosleep и sem_wait (ipc_wait повторяет ожидание).
static void set_signals(void) {
    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    signal(SIGPIPE, SIG_IGN);   // вывод в закрытый канал (| head) не должен убивать процесс
}

// ------------------------------------------------------ независимый запуск (-R)
#define SEP_KEY "/expd"   // постоянное имя объектов IPC для независимых процессов

// Роль участника id для команды запуска: robot:1, dispatcher, node:0, charger:1, base
static void role_str(const World* w, int id, char* buf, size_t n) {
    if (id < w->nrob) snprintf(buf, n, "robot:%d", id + 1);
    else if (id == w->nrob) snprintf(buf, n, "dispatcher");
    else if (id <= w->nrob + w->nnode) snprintf(buf, n, "node:%d", id - w->nrob - 1);
    else if (id < w->npart) snprintf(buf, n, "charger:%d", id - w->nrob - w->nnode);
    else snprintf(buf, n, "base");
}

// Номер участника по роли; -1 — нет такой роли в сценарии
static int role_id(const World* w, const char* role) {
    char buf[24];
    for (int id = 0; id <= w->npart; id++) {
        role_str(w, id, buf, sizeof buf);
        if (!strcmp(buf, role)) return id;
    }
    return -1;
}

// Независимый участник: подключиться к миру, занять роль, выполнять такты
static int participant(const char* role) {
    World* w = ipc_attach(SEP_KEY);
    if (!w) return 1;
    int id = role_id(w, role);
    ipc_lock();
    const char* why = id < 0 ? "такой роли нет в сценарии" : w->stop ? "модель уже остановлена"
                    : w->pid[id] ? "роль уже занята другим процессом" : NULL;
    if (!why) { w->pid[id] = getpid(); w->joined++; }
    ipc_unlock();
    if (why) {
        dprintf(2, "%s: %s\n", role, why);
        ipc_detach(w);
        return 1;
    }
    g_w = w;
    g_echo = true;
    set_signals();
    ipc_post(S_JOIN);
    run_part(w, id);
    g_w = NULL;
    ipc_detach(w);
    dprintf(1, "Процесс %s (pid %d) завершён\n", role, (int)getpid());
    return 0;
}

// Часы: подсказать команды и дождаться всех участников; false — прервано сигналом
static bool wait_participants(World* w) {
    char role[24];
    dprintf(1, "Часы готовы. Запустите участников, каждого своей командой (в любом порядке):\n");
    for (int id = 0; id <= w->npart; id++) {
        role_str(w, id, role, sizeof role);
        dprintf(1, "  ./expedition -R %s\n", role);
    }
    for (int k = 0; k <= w->npart; k++) {
        while (!ipc_wait_once(S_JOIN))           // прерывание сигналом — Ctrl+C у часов
            if (w->signo) return false;
        ipc_lock();
        if (k == w->npart) ev(w, 0, "ОЖИДАНИЕ подключились все участники (%d) — начинаются такты", k + 1);
        flush_events(w);                         // показать подключившихся
        ipc_unlock();
    }
    return true;
}

int main(int argc, char* argv[]) {
    int rc = config_load(&g_init.cfg, argc, argv);
    if (rc) return rc == 2 ? 0 : 1;
    const char* role = g_init.cfg.role;
    bool separate = !strcmp(role, "clock");      // часы независимого запуска
    if (role[0] && !separate && strcmp(role, "list") != 0) return participant(role);
    char err[160];
    if (world_init(&g_init, err, sizeof err) != 0) {
        dprintf(2, "Ошибка сценария: %s\n", err);
        return 1;
    }
    if (!strcmp(role, "list")) {                 // роли участников — для скрипта запуска
        char buf[24];
        for (int id = 0; id <= g_init.npart; id++) {
            role_str(&g_init, id, buf, sizeof buf);
            dprintf(1, "%s\n", buf);
        }
        return 0;
    }
    char key[32];                                // версия с fork — своё имя на каждый запуск
    if (separate) snprintf(key, sizeof key, "%s", SEP_KEY);
    else snprintf(key, sizeof key, "/expd%d", (int)getpid());
    World* w = ipc_create(&g_init, key);
    if (!w) return 1;
    g_w = w;
    if (strcmp(w->cfg.log_path, "none") != 0 &&
        (g_log = open(w->cfg.log_path, O_WRONLY | O_CREAT | O_TRUNC, 0644)) < 0)
        dprintf(2, "Журнал %s не открыт: %s\n", w->cfg.log_path, strerror(errno));
    set_signals();   // дети fork наследуют обработчик
    ipc_lock();
    ev(w, 0, "СТАРТ    процессов: %d — роботы %d, диспетчер 1, узлы связи %d, станции %d, база 1; часы — pid %d%s",
       w->npart + 1, w->nrob, w->nnode, w->nch, (int)getpid(), separate ? ", участники запускаются отдельно" : "");
    flush_events(w);
    ipc_unlock();
    if (separate) {
        if (!wait_participants(w)) w->end = END_SIGNAL;
    } else if (sim_fork(w) != 0) {   // не все процессы созданы: остановить запущенные, удалить IPC
        sim_stop(w);
        ipc_destroy(w);
        return 1;
    }

    for (;;) {
        ipc_lock();
        EndReason e = check_end(w);
        if (e == END_NONE && w->signo) e = w->end = END_SIGNAL;
        flush_events(w);
        ipc_unlock();
        if (e != END_NONE) break;
        sim_tick(w);
        if (w->cfg.map_view) {
            ipc_lock();
            flush_events(w);
            draw(w);
            ipc_unlock();
        }
        struct timespec ts = {w->cfg.delay_ms / 1000, (long)(w->cfg.delay_ms % 1000) * 1000000L};
        if (w->cfg.map_view || w->cfg.delay_ms > 0) nanosleep(&ts, NULL);   // прерывается сигналом
    }
    ipc_lock();
    if (w->end == END_SIGNAL) ev(w, 0, "ВНИМАНИЕ получен сигнал %d — моделирование прервано", (int)w->signo);
    ipc_unlock();
    sim_stop(w);
    ipc_lock();
    flush_events(w);
    ipc_unlock();
    static const int codes[] = {1, 0, 2, 3, 4, 5};
    int code = codes[w->end];
    report(w, 1, code);
    if (g_log >= 0) { report(w, g_log, code); close(g_log); }
    g_w = NULL;
    ipc_destroy(w);   // удалить семафоры и разделяемую память при любом исходе
    return code;
}
