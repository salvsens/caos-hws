// =============================================================================
// main.c — главная программа: журнал событий (экран + файл через дескрипторы),
// отображение карты, цикл моделирования, обработка сигналов, итоговый отчёт.
// =============================================================================
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "model.h"

static World g_world;                        // модель (крупная структура — в статической памяти)
static int g_log = -1;                       // дескриптор файла журнала
static volatile sig_atomic_t g_signal = 0;   // номер полученного сигнала
static char g_recent[10][220];               // последние события для режима карты
static int g_nrecent = 0;

// Событие: в файл журнала — всегда; на экран — уровень 0 (уровень 1 — с -v)
void ev(World* w, int level, const char* fmt, ...) {
    char text[200];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(text, sizeof text, fmt, ap);
    va_end(ap);
    if (g_log >= 0) dprintf(g_log, "[%04d] %s\n", w->tick, text);
    if (level > w->cfg.verbose) return;
    if (w->cfg.map_view) {
        memmove(g_recent[1], g_recent[0], sizeof g_recent - sizeof g_recent[0]);
        snprintf(g_recent[0], sizeof g_recent[0], "[%04d] %s", w->tick, text);
        if (g_nrecent < 10) g_nrecent++;
    } else {
        dprintf(1, "[%04d] %s\n", w->tick, text);
    }
}

static void on_signal(int sig) { g_signal = sig; }

// Кадр режима карты: карта с роботами, состояние роботов и заданий, последние события
static void draw(World* w) {
    dprintf(1, "\033[H\033[2J== Планетарная экспедиция == такт %d (лимит %d), seed %llu\n",
            w->tick, w->cfg.max_ticks, (unsigned long long)w->cfg.seed);
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
        dprintf(1, "  R%d %-8s энергия %3d/%-3d память %d/%d %-14s задание %c %s\n", i + 1, r->name, r->energy,
                r->emax, r->nmem, r->mem_cap, robot_state(r->st), r->task >= 0 ? w->task[r->task].id : '-',
                r->in_comm ? "связь" : "нет связи");
    }
    int done = 0;
    for (int t = 0; t < w->ntask; t++) done += w->task[t].st == T_DONE;
    dprintf(1, "  Задания: выполнено %d из %d | база: пакетов %d, дубликатов %d | в пути %d\n",
            done, w->ntask, w->unique, w->dups, w->nmsg);
    for (int i = g_nrecent - 1; i >= 0; i--) dprintf(1, "  %s\n", g_recent[i]);
}

// Итоговый отчёт: причина, задания, недоставленные данные, статистика
static void report(World* w, int fd, int code) {
    static const char* why[] = {"", "выполнены все доступные задания", "достигнут предел тактов",
                                "ресурсы экспедиции исчерпаны", "прервано пользователем", "НАРУШЕН ИНВАРИАНТ"};
    static const char* st[] = {"ожидает", "назначено", "выполняется", "ВЫПОЛНЕНО", "недоступно"};
    dprintf(fd, "\n===== ИТОГИ ЭКСПЕДИЦИИ =====\nПричина завершения: %s %s\nТактов: %d, seed: %llu\n",
            why[w->end], w->end == END_INVARIANT ? w->violation : "", w->tick, (unsigned long long)w->cfg.seed);
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
        dprintf(fd, "  R%d %-8s %-12s энергия %d/%d, шагов %d, передач %d (повторов %d), получено энергии %d\n",
                i + 1, r->name, robot_state(r->st), r->energy, r->emax, r->moves, r->sent, r->retries, r->charged);
    }
    dprintf(fd, "Связь: отправлено назначений/пакетов/подтверждений %d/%d/%d, потеряно %d/%d/%d, продублировано %d/%d/%d\n",
            w->sent[0], w->sent[1], w->sent[2], w->lost[0], w->lost[1], w->lost[2], w->dupd[0], w->dupd[1], w->dupd[2]);
    dprintf(fd, "База: засчитано уникальных пакетов %d, отброшено дубликатов %d\n", w->unique, w->dups);
    dprintf(fd, "Инварианты: проверок %d, %s\nКод завершения: %d\n", w->checks,
            w->end == END_INVARIANT ? "есть нарушение" : "нарушений нет", code);
}

int main(int argc, char* argv[]) {
    World* w = &g_world;
    int rc = config_load(&w->cfg, argc, argv);
    if (rc) return rc == 2 ? 0 : 1;
    rnd_seed(w->cfg.seed);
    if (strcmp(w->cfg.log_path, "none") != 0 &&
        (g_log = open(w->cfg.log_path, O_WRONLY | O_CREAT | O_TRUNC, 0644)) < 0)
        dprintf(2, "Журнал %s не открыт: %s\n", w->cfg.log_path, strerror(errno));
    char err[160];
    if (world_init(w, err, sizeof err) != 0) {
        dprintf(2, "Ошибка сценария: %s\n", err);
        return 1;
    }
    // Ctrl+C, SIGTERM: обработчик только ставит флаг, цикл завершается корректно
    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    while (check_end(w) == END_NONE) {
        if (g_signal) { w->end = END_SIGNAL; break; }
        world_step(w);
        if (w->cfg.map_view) draw(w);
        struct timespec ts = {w->cfg.delay_ms / 1000, (long)(w->cfg.delay_ms % 1000) * 1000000L};
        if (w->cfg.map_view || w->cfg.delay_ms > 0) nanosleep(&ts, NULL);   // прерывается сигналом
    }
    if (w->end == END_SIGNAL) ev(w, 0, "ВНИМАНИЕ получен сигнал %d — моделирование прервано", (int)g_signal);
    static const int codes[] = {1, 0, 2, 3, 4, 5};
    int code = codes[w->end];
    report(w, 1, code);
    if (g_log >= 0) { report(w, g_log, code); close(g_log); }
    return code;
}
