// =============================================================================
// main.c — команды запуска экспедиции, клиенты и обработка сигналов.
// =============================================================================
#include <arpa/inet.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include "protocol.h"

volatile sig_atomic_t interrupted;
static World world;

static void on_signal(int sig) { interrupted = sig; }

static void set_signals(void) {
    struct sigaction sa = {.sa_handler = on_signal};
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    signal(SIGPIPE, SIG_IGN);
}

static int init_world(void) {
    char err[160];
    if (world_init(&world, err, sizeof err) == 0) return 0;
    dprintf(2, "Ошибка сценария: %s\n", err);
    return -1;
}

static int client(int fd, uint32_t role, unsigned number, unsigned flags) {
    uint32_t request[] = {htonl(role == F_WATCH ? flags : number), htonl(flags)};
    if (tcp_send(fd, role, request, role == F_CONTROL ? sizeof request : sizeof request[0]) < 0)
        return interrupted ? 4 : 1;
    Frame f = {0};
    bool configured = false;
    int id = (int)number - 1;
    while (!interrupted) {
        int rc = tcp_read(fd, &f);
        if (rc < 0) break;
        if (!rc) continue;
        if (f.type == F_END && f.len == sizeof(uint32_t)) {
            uint32_t result;
            memcpy(&result, f.data, sizeof result);
            result = ntohl(result);
            if (result > 5) return 1;
            dprintf(1, "Сервер завершил работу, код %u\n", result);
            return (int)result;
        }
        if (f.type == F_ERROR) {
            dprintf(2, "Сервер: %.*s\n", (int)f.len, (const char*)f.data);
            return 1;
        }
        if (role == F_CONTROL && f.type == F_CONTROL_OK) {
            dprintf(1, "%.*s\n", (int)f.len, (const char*)f.data);
            return 0;
        }
        if (role == F_ROBOT && f.type == F_KICK && !f.len) {
            dprintf(1, "КЛИЕНТ   R%u отключён командой управления\n", number);
            return 0;
        }
        if (role == F_WATCH && (f.type == F_MAP || f.type == F_EVENT || f.type == F_REPORT)) {
            if (f.type == F_MAP && (flags & WATCH_MAP) && isatty(1)) dprintf(1, "\033[H\033[2J");
            dprintf(1, "%.*s%s", (int)f.len, (const char*)f.data, f.type == F_EVENT ? "\n" : "");
        } else if (role == F_ROBOT && !configured && f.type == F_CONFIG && f.len == sizeof(Config)) {
            memcpy(&world.cfg, f.data, sizeof world.cfg);
            if (world.cfg.nrob < (int)number || world.cfg.nrob > MAXROB ||
                world.cfg.rows < 3 || world.cfg.rows > MAXR ||
                (unsigned)world.cfg.ntask > MAXT || init_world() < 0) return 1;
            configured = true;
            dprintf(1, "КЛИЕНТ   R%u подключён\n", number);
        } else if (configured && f.type == F_TURN && f.len == sizeof(Turn)) {
            Turn turn;
            Step step;
            memcpy(&turn, f.data, sizeof turn);
            turn_run(&world, id, &turn, &step);
            int rc = tcp_send(fd, F_STEP, &step, sizeof step);
            if (rc < 0) break;
            for (int i = 0; i < step.nev; i++)
                if ((unsigned)step.events[i].level <= flags) dprintf(1, "%s\n", step.events[i].text);
        } else {
            dprintf(2, "Неожиданное сообщение сервера\n");
            return 1;
        }
    }
    if (interrupted) { dprintf(1, "Клиент отключён сигналом\n"); return 4; }
    dprintf(2, "Соединение с сервером закрыто без итога\n");
    return 1;
}

static void usage(int fd) {
    dprintf(fd, "Использование:\n"
            "  ./expedition server IP PORT [опции сценария]\n"
            "  ./expedition robot IP PORT N [-v]\n"
            "  ./expedition watch IP PORT [-m] [-v]\n"
            "  ./expedition control IP PORT off|on N\n"
            "Справка по сценарию: ./expedition server -h\n");
}

static unsigned robot_number(const char* name) {
    return name[0] >= '1' && name[0] <= '0' + MAXROB && !name[1] ? (unsigned)(name[0] - '0') : 0;
}

static int connect_client(const char* ip, const char* port, uint32_t role, unsigned number, unsigned flags) {
    int fd = tcp_connect(ip, port);
    if (fd < 0) {
        if (interrupted) return 4;
        dprintf(2, "Не удалось подключиться к %s:%s: %s\n", ip, port, strerror(errno));
        return 1;
    }
    struct timeval in = {.tv_usec = 200000}, out = {.tv_sec = 2};
    if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &in, sizeof in) < 0 ||
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &out, sizeof out) < 0) {
        dprintf(2, "Не удалось настроить сокет: %s\n", strerror(errno));
        close(fd);
        return 1;
    }
    int rc = client(fd, role, number, flags);
    close(fd);
    return rc;
}

int main(int argc, char* argv[]) {
    if (argc == 2 && !strcmp(argv[1], "-h")) { usage(1); return 0; }
    if (argc == 3 && !strcmp(argv[1], "server") && !strcmp(argv[2], "-h")) {
        config_load(&world.cfg, argc - 1, argv + 1);
        return 0;
    }
    set_signals();
    if (argc >= 4 && !strcmp(argv[1], "server")) {
        int rc = config_load(&world.cfg, argc - 3, argv + 3);
        if (rc) return rc == 2 ? 0 : 1;
        if (init_world() < 0) return 1;
        return server_run(&world, argv[2], argv[3]);
    }
    if ((argc == 5 || (argc == 6 && !strcmp(argv[5], "-v"))) && !strcmp(argv[1], "robot")) {
        unsigned number = robot_number(argv[4]);
        if (number) return connect_client(argv[2], argv[3], F_ROBOT, number, argc == 6);
    }
    if (argc >= 4 && !strcmp(argv[1], "watch")) {
        unsigned flags = 0;
        for (int i = 4; i < argc; i++) {
            if (!strcmp(argv[i], "-m")) flags |= WATCH_MAP;
            else if (!strcmp(argv[i], "-v")) flags |= WATCH_VERBOSE;
            else { usage(2); return 1; }
        }
        return connect_client(argv[2], argv[3], F_WATCH, 0, flags);
    }
    if (argc == 6 && !strcmp(argv[1], "control") && (!strcmp(argv[4], "on") || !strcmp(argv[4], "off"))) {
        unsigned number = robot_number(argv[5]);
        if (number) return connect_client(argv[2], argv[3], F_CONTROL, number, !strcmp(argv[4], "on"));
    }
    usage(2);
    return 1;
}
