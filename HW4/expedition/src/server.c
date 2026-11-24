// =============================================================================
// server.c — сервер экспедиции: соединения TCP, очереди вывода и часы модели.
// =============================================================================
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include "protocol.h"

enum { WATCHERS = 8, PEERS = MAXROB + WATCHERS + 4, TX_CAP = 65536, IO_MS = 2000 };

typedef struct {
    int fd, robot, watch;             // номер робота или флаги наблюдателя (-1 — нет)
    Frame in;
    unsigned char out[TX_CAP];
    size_t off, end;
    int64_t closing;                  // срок закрытия после последнего сообщения
} Peer;

typedef struct {
    World* w;
    int listener, log, waiting;       // waiting — соединение, от которого ждём ответ
    int robot[MAXROB];                // индексы соединений (-1 — робота нет)
    bool seen[MAXROB], blocked[MAXROB];
    int joined, watchers;             // роботы, входившие хотя бы раз, и текущие наблюдатели
    unsigned shown;
    bool ending, failure;
    Peer peers[PEERS];
} Server;

static int64_t now_ms(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000 + t.tv_nsec / 1000000;
}

static void robot_left(Server* s, int i) {
    Peer* p = &s->peers[i];
    int id = p->robot;
    if (id < 0) return;
    s->robot[id] = -1;
    s->w->online[id] = false;
    if (s->w->rob[id].st == R_CHARGE) s->w->rob[id].st = R_QUEUE;
    p->robot = -1;
    if (s->waiting == i) s->waiting = -1;
    if (!s->ending) {
        dprintf(1, "СЕРВЕР   R%d отключён\n", id + 1);
        ev(s->w, 0, "TCP      R%d отключён", id + 1);
    }
}

static void drop(Server* s, int i) {
    Peer* p = &s->peers[i];
    robot_left(s, i);
    if (p->watch >= 0) {
        s->watchers--;
        if (!s->ending) {
            dprintf(1, "СЕРВЕР   наблюдатель отключён\n");
            ev(s->w, 0, "TCP      наблюдатель отключён");
        }
    }
    close(p->fd);
    p->fd = -1;
}

static bool queue(Peer* p, uint32_t type, const void* data, size_t len) {
    size_t size = TCP_HEADER + len;
    if (len > TCP_PAYLOAD || size > sizeof p->out - (p->end - p->off)) return false;
    if (size > sizeof p->out - p->end) {
        memmove(p->out, p->out + p->off, p->end - p->off);
        p->end -= p->off;
        p->off = 0;
    }
    p->end += tcp_pack(p->out + p->end, type, data, len);
    return true;
}

static void finish_peer(Server* s, int i, uint32_t type, const void* data, size_t len) {
    Peer* p = &s->peers[i];
    if (queue(p, type, data, len)) p->closing = now_ms() + IO_MS;
    else drop(s, i);
}

static uint32_t word(const Frame* f, size_t i) {
    uint32_t value;
    memcpy(&value, f->data + i * sizeof value, sizeof value);
    return ntohl(value);
}

static void control(Server* s, int i, int id, bool enable) {
    s->blocked[id] = !enable;
    int link = s->robot[id];
    if (!enable && link >= 0) {
        robot_left(s, link);
        finish_peer(s, link, F_KICK, NULL, 0);
    }
    char msg[128];
    snprintf(msg, sizeof msg, "R%d: %s", id + 1,
             enable ? "вход разрешён" : "отключён, вход запрещён");
    dprintf(1, "СЕРВЕР   управление: %s\n", msg);
    ev(s->w, 0, "TCP      управление: %s", msg);
    finish_peer(s, i, F_CONTROL_OK, msg, strlen(msg));
}

static void receive(Server* s, int i) {
    Peer* p = &s->peers[i];
    World* w = s->w;
    if (p->closing) {
        unsigned char drain[1024];
        ssize_t n = read(p->fd, drain, sizeof drain);
        if (!n || (n < 0 && errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK)) drop(s, i);
        return;
    }
    int rc = tcp_read(p->fd, &p->in);
    if (rc < 0) { drop(s, i); return; }
    if (!rc || s->ending) return;
    Frame* f = &p->in;
    const char* error = "неожиданное сообщение";
    bool new_peer = p->robot < 0 && p->watch < 0;
    if (new_peer && f->type == F_ROBOT && f->len == sizeof(uint32_t)) {
        uint32_t number = word(f, 0);
        if (!number || number > (unsigned)w->nrob) error = "такого робота нет в сценарии";
        else {
            int id = (int)number - 1;
            if (s->blocked[id]) error = "вход этого робота запрещён";
            else if (s->robot[id] >= 0) error = "роль уже занята";
            else if (queue(p, F_CONFIG, &w->cfg, sizeof w->cfg)) {
                p->robot = id;
                s->robot[id] = i;
                w->online[id] = true;
                bool again = s->seen[id];
                if (!again) { s->seen[id] = true; s->joined++; }
                const char* action = again ? "подключился снова" : "подключился";
                dprintf(1, "СЕРВЕР   R%d %s\n", id + 1, action);
                ev(w, 0, "TCP      R%d %s", id + 1, action);
                return;
            }
        }
    } else if (new_peer && f->type == F_WATCH && f->len == sizeof(uint32_t)) {
        uint32_t flags = word(f, 0);
        if (flags > (WATCH_MAP | WATCH_VERBOSE)) error = "неверный режим наблюдения";
        else if (s->watchers == WATCHERS) error = "достигнут предел наблюдателей";
        else {
            Text map;
            draw(w, &map, (flags & WATCH_VERBOSE) != 0);
            if (queue(p, F_MAP, map.data, map.len)) {
                p->watch = (int)flags;
                s->watchers++;
                dprintf(1, "СЕРВЕР   наблюдатель подключён\n");
                ev(w, 0, "TCP      наблюдатель подключён");
                return;
            }
        }
    } else if (new_peer && f->type == F_CONTROL && f->len == 2 * sizeof(uint32_t)) {
        uint32_t number = word(f, 0), enable = word(f, 1);
        if (!number || number > (unsigned)w->nrob || enable > 1) error = "неверная команда управления";
        else { control(s, i, (int)number - 1, enable != 0); return; }
    } else if (s->waiting == i && f->type == F_STEP && f->len == sizeof(Step)) {
        Step step;
        memcpy(&step, f->data, sizeof step);
        if (turn_apply(w, p->robot, &step)) {
            if (w->cfg.verbose) dprintf(1, "СЕРВЕР   такт %d: принят шаг R%d\n", w->tick, p->robot + 1);
            s->waiting = -1;
            return;
        }
        error = "неверный результат шага";
    }
    robot_left(s, i);
    dprintf(2, "СЕРВЕР   %s\n", error);
    finish_peer(s, i, F_ERROR, error, strlen(error));
}

static void accept_peer(Server* s) {
    int fd = accept(s->listener, NULL, NULL);
    if (fd < 0) return;
    int i = 0;
    while (i < PEERS && s->peers[i].fd >= 0) i++;
    if (i == PEERS || tcp_nonblock(fd) < 0) { close(fd); return; }
    Peer* p = &s->peers[i];
    p->fd = fd;
    p->robot = p->watch = -1;
    p->in.used = p->off = p->end = 0;
    p->closing = 0;
}

static void pump(Server* s, int timeout) {
    struct pollfd fds[PEERS + 1];
    fds[0] = (struct pollfd){.fd = s->listener, .events = POLLIN};
    for (int i = 0; i < PEERS; i++) {
        Peer* p = &s->peers[i];
        if (p->fd >= 0 && p->closing && now_ms() >= p->closing) drop(s, i);
        fds[i + 1] = (struct pollfd){.fd = p->fd, .events = POLLIN | (p->off < p->end ? POLLOUT : 0)};
    }
    int rc = poll(fds, PEERS + 1, timeout > 200 ? 200 : timeout);
    if (rc < 0 && errno != EINTR) {
        dprintf(2, "Ошибка poll: %s\n", strerror(errno));
        s->failure = true;
    }
    if (rc <= 0) return;
    for (int i = 0; i < PEERS; i++) {
        Peer* p = &s->peers[i];
        short ready = fds[i + 1].revents;
        if (p->fd < 0) continue;
        if (ready & POLLIN) receive(s, i);
        if (p->fd < 0) continue;
        if (ready & (POLLERR | POLLHUP | POLLNVAL)) { drop(s, i); continue; }
        if ((ready & POLLOUT) && p->off < p->end) {
            ssize_t n = write(p->fd, p->out + p->off, p->end - p->off);
            if (n > 0) {
                p->off += (size_t)n;
                if (p->off == p->end) {
                    p->off = p->end = 0;
                    if (p->closing) shutdown(p->fd, SHUT_WR);
                }
            } else if (!n || (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK)) drop(s, i);
        }
    }
    if (fds[0].revents & POLLIN) accept_peer(s);
}

static void publish(Server* s, bool tick) {
    World* w = s->w;
    bool refresh = tick || s->shown != w->ev_head;
    if (w->ev_head - s->shown > EVQ) {
        if (s->log >= 0) dprintf(s->log, "... пропущено событий: %u\n", w->ev_head - s->shown - EVQ);
        s->shown = w->ev_head - EVQ;
    }
    for (; s->shown != w->ev_head; s->shown++) {
        const Event* e = &w->evq[s->shown % EVQ];
        if (s->log >= 0) dprintf(s->log, "%s\n", e->text);
        for (int i = 0; i < PEERS; i++) {
            Peer* p = &s->peers[i];
            if (p->fd >= 0 && p->watch >= 0 && !(p->watch & WATCH_MAP) &&
                (e->level == 0 || (p->watch & WATCH_VERBOSE)) &&
                !queue(p, F_EVENT, e->text, strlen(e->text))) drop(s, i);
        }
    }
    if (!refresh) return;
    Text map[2];
    bool rendered[2] = {false, false};
    for (int i = 0; i < PEERS; i++) {
        Peer* p = &s->peers[i];
        if (p->fd < 0 || p->watch < 0 || !(p->watch & WATCH_MAP)) continue;
        int v = (p->watch & WATCH_VERBOSE) != 0;
        if (!rendered[v]) { draw(w, &map[v], v); rendered[v] = true; }
        if (!queue(p, F_MAP, map[v].data, map[v].len)) drop(s, i);
    }
}

static void robot_step(Server* s, int id) {
    int i = s->robot[id];
    if (i < 0) return;
    Turn turn;
    turn_make(s->w, id, &turn);
    if (!queue(&s->peers[i], F_TURN, &turn, sizeof turn)) { drop(s, i); return; }
    s->waiting = i;
    int64_t until = now_ms() + IO_MS;
    while (!interrupted && !s->failure && s->waiting == i) {
        int left = (int)(until - now_ms());
        if (left <= 0) {
            ev(s->w, 0, "TCP      R%d: истекло ожидание ответа", id + 1);
            drop(s, i);
            break;
        }
        pump(s, left);
    }
}

static bool active(const Server* s) {
    for (int i = 0; i < s->w->nrob; i++) if (s->robot[i] >= 0) return true;
    return false;
}

static void run_ticks(Server* s) {
    World* w = s->w;
    while (s->joined < w->nrob && !interrupted && !s->failure) {
        pump(s, 200);
        publish(s, false);
    }
    if (s->joined == w->nrob) {
        dprintf(1, "СЕРВЕР   подключились все роботы (%d)\n", w->nrob);
        ev(w, 0, "TCP      подключились все роботы (%d)", w->nrob);
    }
    while (!interrupted && !s->failure && check_end(w) == END_NONE) {
        pump(s, 0);
        if (interrupted || s->failure) break;
        if (!active(s)) { pump(s, 200); publish(s, false); continue; }
        w->tick++;
        dispatcher_tick(w);
        for (int k = 0; k < w->nnode; k++) node_tick(w, k);
        base_tick(w);
        for (int j = 0; j < w->nrob && !interrupted && !s->failure; j++)
            robot_step(s, (w->tick - 1 + j) % w->nrob);
        for (int c = 0; c < w->nch; c++) charger_tick(w, c);
        if (!invariants(w)) w->end = END_INVARIANT;
        publish(s, true);
        int64_t until = now_ms() + w->cfg.delay_ms;
        for (int left; !interrupted && !s->failure && (left = (int)(until - now_ms())) > 0;)
            pump(s, left);
    }
}

int server_run(World* w, const char* ip, const char* port) {
    static Server s;
    s.w = w;
    s.log = s.waiting = -1;
    for (int i = 0; i < PEERS; i++) s.peers[i].fd = -1;
    for (int i = 0; i < MAXROB; i++) s.robot[i] = -1;
    s.listener = tcp_listen(ip, port);
    if (s.listener < 0 || tcp_nonblock(s.listener) < 0) {
        dprintf(2, "Не удалось открыть %s:%s: %s\n", ip, port, strerror(errno));
        if (s.listener >= 0) close(s.listener);
        return 1;
    }
    if (strcmp(w->cfg.log_path, "none")) {
        s.log = open(w->cfg.log_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (s.log < 0) dprintf(2, "Журнал %s не открыт: %s\n", w->cfg.log_path, strerror(errno));
    }
    dprintf(1, "СЕРВЕР   готов %s:%s, роботов в сценарии: %d\n", ip, port, w->nrob);
    ev(w, 0, "СТАРТ    сервер %s:%s; роботы %d, узлы связи %d, станции %d", ip, port, w->nrob, w->nnode, w->nch);
    publish(&s, false);
    run_ticks(&s);

    if (interrupted && w->end == END_NONE) {
        w->end = END_SIGNAL;
        ev(w, 0, "ВНИМАНИЕ получен сигнал %d — моделирование прервано", (int)interrupted);
    }
    static const int codes[] = {1, 0, 2, 3, 4, 5};
    int code = codes[w->end];
    publish(&s, false);
    Text result_text;
    report(w, &result_text, code);
    if (s.log >= 0) { dprintf(s.log, "%s", result_text.data); close(s.log); }
    dprintf(1, "СЕРВЕР   завершение на такте %d, код %d\n", w->tick, code);
    close(s.listener);
    s.listener = -1;
    s.ending = true;

    uint32_t result = htonl((uint32_t)code);
    for (int i = 0; i < PEERS; i++) {
        Peer* p = &s.peers[i];
        if (p->fd < 0 || p->closing) continue;
        if (p->watch >= 0 && !queue(p, F_REPORT, result_text.data, result_text.len)) drop(&s, i);
        else finish_peer(&s, i, F_END, &result, sizeof result);
    }

    int64_t until = now_ms() + IO_MS;
    for (bool any = true; any && !s.failure && now_ms() < until;) {
        any = false;
        for (int i = 0; i < PEERS; i++) any |= s.peers[i].fd >= 0;
        if (any) pump(&s, (int)(until - now_ms()));
    }
    for (int i = 0; i < PEERS; i++) if (s.peers[i].fd >= 0) drop(&s, i);
    return code;
}
