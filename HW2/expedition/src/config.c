// =============================================================================
// config.c — параметры моделирования: встроенный сценарий, файл сценария
// (open/read), аргументы командной строки (getopt).
// =============================================================================
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include "model.h"

// Встроенный демонстрационный сценарий (тот же формат, что у файла сценария)
static const char DEMO[] =
    "seed = 7\nmax_ticks = 500\nstrategy = nearest\ndelay_ms = 120\n"
    "p_loss = 0.15\np_dup = 0.08\nnet_delay = 1\nnet_jitter = 2\nack_timeout = 8\n"
    "danger_fail = 0.03\n"
    "robot = Rover-1 CAM+SPEC 100 6\n"
    "robot = Rover-2 DRILL+CAM 90 6\n"
    "robot = Rover-3 SEIS+RAD 90 5\n"
    "robot = Rover-4 CAM+RAD 60 4\n"
    "target = a CAM 3 2\ntarget = b SPEC 3 3\ntarget = c CAM 2 2\ntarget = d SEIS 3 2\n"
    "target = e DRILL 4 3\ntarget = f RAD 3 2\ntarget = g CAM+SPEC 3 2\ntarget = h RAD 2 2\n"
    "target = k MAG 2 1\n"
    "map:\n"
    "########################################\n"
    "#B......#.......~~.......a.........b...#\n"
    "#..C....#.......~~.....................#\n"
    "#.......#...c...~~.......^.............#\n"
    "#..d...........................C.......#\n"
    "#.......####...~~~......####...........#\n"
    "#...e...#......~~~......#.f#.......g...#\n"
    "#.......#..........^.......#...........#\n"
    "#~~~~~..#..h...........................#\n"
    "#..k....#..............................#\n"
    "########################################\n"
    "end\n";

static const char* INAMES[] = {"CAM", "SPEC", "DRILL", "RAD", "SEIS", "MAG"};

// "CAM+SPEC" -> битовая маска; ~0u — неизвестный прибор
unsigned parse_instr(const char* s) {
    char buf[64];
    unsigned m = 0;
    snprintf(buf, sizeof buf, "%s", s);
    for (char* save = NULL, *tok = strtok_r(buf, "+", &save); tok; tok = strtok_r(NULL, "+", &save)) {
        int k = 0;
        while (k < 6 && strcasecmp(tok, INAMES[k]) != 0) k++;
        if (k == 6) return ~0u;
        m |= 1u << k;
    }
    return m;
}

void format_instr(unsigned m, char* buf, size_t n) {
    buf[0] = '\0';
    for (int k = 0; k < 6; k++)
        if (m & (1u << k)) {
            size_t len = strlen(buf);
            snprintf(buf + len, n - len, "%s%s", len ? "+" : "", INAMES[k]);
        }
}

// Число из строки целиком; false — ошибка
static bool num(const char* s, double* out) {
    char* end;
    errno = 0;
    *out = strtod(s, &end);
    return errno == 0 && end != s && *end == '\0';
}

// Установка параметра «ключ = значение»; -1 — ошибка
static int set_param(Config* c, const char* k, const char* v) {
    double x;
    if (!strcmp(k, "strategy")) {
        if (!strcmp(v, "fifo")) c->strategy = 0;
        else if (!strcmp(v, "nearest")) c->strategy = 1;
        else return -1;
        return 0;
    }
    if (!num(v, &x)) return -1;
    // Целые параметры: имя -> поле структуры
    struct { const char* key; int* field; int min, max; } ints[] = {
        {"max_ticks", &c->max_ticks, 0, 1000000}, {"delay_ms", &c->delay_ms, 0, 10000},
        {"move_cost", &c->move_cost, 1, 100}, {"danger_cost", &c->danger_cost, 0, 100},
        {"measure_cost", &c->measure_cost, 0, 100}, {"tx_cost", &c->tx_cost, 0, 100},
        {"recharge_pct", &c->recharge_pct, 0, 100}, {"charge_rate", &c->charge_rate, 1, 1000},
        {"charger_stock", &c->charger_stock, -1, 1000000}, {"charger_slots", &c->charger_slots, 1, 8},
        {"net_delay", &c->net_delay, 1, 100},
        {"net_jitter", &c->net_jitter, 0, 100}, {"ack_timeout", &c->ack_timeout, 1, 1000},
        {"base_radius", &c->base_radius, 0, 60}, {"relay_radius", &c->relay_radius, 0, 60},
        {"assign_timeout", &c->assign_timeout, 1, 1000}, {"task_timeout", &c->task_timeout, 1, 100000},
        {"stall_ticks", &c->stall_ticks, 10, 1000000}};
    for (size_t i = 0; i < sizeof ints / sizeof ints[0]; i++)
        if (!strcmp(k, ints[i].key)) {
            if (x != (int)x || x < ints[i].min || x > ints[i].max) return -1;
            *ints[i].field = (int)x;
            return 0;
        }
    struct { const char* key; double* field; } dbls[] = {
        {"p_loss", &c->p_loss}, {"p_dup", &c->p_dup}, {"danger_fail", &c->danger_fail}};
    for (size_t i = 0; i < 3; i++)
        if (!strcmp(k, dbls[i].key)) {
            if (x < 0 || x > 1) return -1;
            *dbls[i].field = x;
            return 0;
        }
    if (!strcmp(k, "seed") && x >= 0) { c->seed = (uint64_t)x; return 0; }
    return -1;
}

// Разбор текста сценария (буфер изменяется). origin — имя для сообщений об ошибках
static int parse_text(Config* c, char* text, const char* origin) {
    bool in_map = false, map_seen = false, robots_set = false, targets_set = false;
    int line_no = 0;
    for (char* save = NULL, *line = strtok_r(text, "\n", &save); line; line = strtok_r(NULL, "\n", &save)) {
        line_no++;
        line[strcspn(line, "\r")] = '\0';
        if (in_map) {                                   // строки карты — как есть
            if (!strcmp(line, "end")) in_map = false;
            else if (c->rows < MAXR && strlen(line) <= MAXC) strcpy(c->map[c->rows++], line);
            else { dprintf(2, "%s:%d: карта больше %dx%d\n", origin, line_no, MAXR, MAXC); return -1; }
            continue;
        }
        line[strcspn(line, "#")] = '\0';                // комментарий
        char key[32], v1[64], v2[64], v3[64], v4[64];
        if (sscanf(line, " %31s", key) != 1) continue;  // пустая строка
        if (!strcmp(key, "map:")) { in_map = map_seen = true; c->rows = 0; continue; }
        char* eq = strchr(line, '=');
        if (!eq) { dprintf(2, "%s:%d: ожидается «ключ = значение»\n", origin, line_no); return -1; }
        sscanf(line, " %31[^= ]", key);
        int n = sscanf(eq + 1, "%63s %63s %63s %63s", v1, v2, v3, v4);
        if (!strcmp(key, "robot")) {                    // robot = ИМЯ ПРИБОРЫ ЭНЕРГИЯ ПАМЯТЬ
            if (!robots_set) { c->nrob = 0; robots_set = true; }
            int i = c->nrob;
            if (n != 4 || i >= MAXROB || (c->rinstr[i] = parse_instr(v2)) == ~0u ||
                (c->remax[i] = atoi(v3)) < 1 || (c->rmem[i] = atoi(v4)) < 1 || c->rmem[i] > MAXMEM) {
                dprintf(2, "%s:%d: формат: robot = ИМЯ ПРИБОРЫ ЭНЕРГИЯ ПАМЯТЬ(1..%d)\n", origin, line_no, MAXMEM);
                return -1;
            }
            snprintf(c->rname[i], 16, "%s", v1);
            c->nrob++;
        } else if (!strcmp(key, "target")) {           // target = БУКВА ПРИБОРЫ ТАКТЫ ПАКЕТЫ
            if (!targets_set) { c->ntask = 0; targets_set = true; }
            int i = c->ntask;
            if (n != 4 || i >= MAXT || strlen(v1) != 1 || v1[0] < 'a' || v1[0] > 'z' ||
                (c->tinstr[i] = parse_instr(v2)) == ~0u || (c->tticks[i] = atoi(v3)) < 1 ||
                (c->tvol[i] = atoi(v4)) < 1 || c->tvol[i] > MAXMEM) {
                dprintf(2, "%s:%d: формат: target = БУКВА ПРИБОРЫ ТАКТЫ ПАКЕТЫ(1..%d)\n", origin, line_no, MAXMEM);
                return -1;
            }
            c->tid[i] = v1[0];
            c->ntask++;
        } else if (n != 1 || set_param(c, key, v1) != 0) {
            dprintf(2, "%s:%d: неизвестный параметр или недопустимое значение: %s\n", origin, line_no, key);
            return -1;
        }
    }
    if (in_map) { dprintf(2, "%s: нет строки end после map:\n", origin); return -1; }
    if (map_seen && !targets_set) c->ntask = 0;   // цели встроенного сценария к новой карте не относятся
    return 0;
}

// Загрузка файла сценария системными вызовами open/read/close
static int load_file(Config* c, const char* path) {
    static char buf[65536];
    int fd = open(path, O_RDONLY);
    if (fd < 0) { dprintf(2, "Не удалось открыть %s: %s\n", path, strerror(errno)); return -1; }
    ssize_t total = 0, n;
    while ((n = read(fd, buf + total, sizeof buf - 1 - total)) > 0) total += n;
    close(fd);
    if (n < 0) { dprintf(2, "Ошибка чтения %s\n", path); return -1; }
    buf[total] = '\0';
    return parse_text(c, buf, path);
}

static void usage(const char* prog) {
    dprintf(1,
        "Использование: %s [-c файл] [-s seed] [-t такты] [-d мс] [-S fifo|nearest] [-l журнал] [-m] [-v]\n"
        "  -c  файл сценария (по умолчанию встроенный)   -s  начальное значение ГСЧ\n"
        "  -t  предельное число тактов (0 — без ограничения, выход по Ctrl+C)\n"
        "  -d  задержка такта, мс   -S  стратегия назначения   -l  журнал (none — не вести)\n"
        "  -m  режим карты (перерисовка экрана)   -v  выводить каждый шаг роботов\n"
        "Коды завершения: 0 — выполнены все доступные задания, 1 — ошибка параметров,\n"
        "  2 — лимит тактов, 3 — ресурсы исчерпаны, 4 — прервано сигналом, 5 — нарушен инвариант\n", prog);
}

// Порядок: значения по умолчанию -> встроенный сценарий или файл -> опции.
// Возврат: 0 — продолжать, 1 — ошибка, 2 — выведена справка
int config_load(Config* c, int argc, char* argv[]) {
    memset(c, 0, sizeof *c);
    c->move_cost = 1; c->danger_cost = 3; c->measure_cost = 2; c->tx_cost = 1;
    c->recharge_pct = 30; c->charge_rate = 10; c->charger_stock = -1; c->charger_slots = 1;
    c->net_delay = 1; c->net_jitter = 1; c->ack_timeout = 6; c->base_radius = 5; c->relay_radius = 5;
    c->assign_timeout = 8; c->task_timeout = 60; c->stall_ticks = 250; c->max_ticks = 500;
    c->strategy = 1; c->delay_ms = 100;
    strcpy(c->log_path, "expedition.log");
    static char demo[sizeof DEMO];
    memcpy(demo, DEMO, sizeof DEMO);
    parse_text(c, demo, "встроенный сценарий");

    const char* file = NULL;
    const char *s = NULL, *t = NULL, *d = NULL, *S = NULL;
    int ch;
    while ((ch = getopt(argc, argv, "c:s:t:d:S:l:mvh")) != -1) {
        switch (ch) {
        case 'c': file = optarg; break;
        case 's': s = optarg; break;
        case 't': t = optarg; break;
        case 'd': d = optarg; break;
        case 'S': S = optarg; break;
        case 'l': snprintf(c->log_path, sizeof c->log_path, "%s", optarg); break;
        case 'm': c->map_view = true; break;
        case 'v': c->verbose = 1; break;
        case 'h': usage(argv[0]); return 2;
        default: usage(argv[0]); return 1;
        }
    }
    if (optind < argc) { dprintf(2, "Лишний аргумент: %s (файл задаётся через -c)\n", argv[optind]); return 1; }
    if (file && load_file(c, file) != 0) return 1;
    if ((s && set_param(c, "seed", s)) || (t && set_param(c, "max_ticks", t)) ||
        (d && set_param(c, "delay_ms", d)) || (S && set_param(c, "strategy", S))) {
        dprintf(2, "Недопустимое значение опции\n");
        return 1;
    }
    if (c->nrob == 0 || c->rows < 3) { dprintf(2, "В сценарии нет роботов или карты\n"); return 1; }
    return 0;
}
