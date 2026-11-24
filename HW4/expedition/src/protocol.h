#ifndef PROTOCOL_H
#define PROTOCOL_H
// =============================================================================
// protocol.h — сообщения TCP между сервером и рабочим клиентом (ИДЗ №4).
// Заголовок и одиночные числа — сетевой порядок. Снимки модели — одна сборка C.
// =============================================================================
#include <signal.h>
#include "model.h"
#include "tcp.h"

enum { F_ROBOT = 0x45585034, F_CONFIG, F_TURN, F_STEP, F_END, F_ERROR,
       F_WATCH, F_MAP, F_EVENT, F_REPORT, F_CONTROL, F_CONTROL_OK, F_KICK };
enum { WATCH_MAP = 1, WATCH_VERBOSE = 2 };
enum { STEP_EVENTS = 32, STEP_TX = 2 };

typedef struct { char data[TCP_PAYLOAD]; size_t len; } Text;

typedef struct {
    int tick, next_uid;
    Robot rob[MAXROB];                        // позиции, зарядка и состояние роботов
    struct { TaskState st; int robot; unsigned unable; } task[MAXT];
    int stock[MAXCH], queued[MAXNODE];        // запас станций и заполненность каналов
    uint8_t delivered[MAXMEM];                // отметки базы по пакетам в памяти этого робота
} Turn;

typedef struct {
    Robot rob;
    int tick, next_uid, progress, abandoned;  // прогресс и отказ от задания (-1 — нет)
    unsigned yielded;                         // кому предложено уступить клетку
    int ntx, nev;
    struct { int uid, at; } tx[STEP_TX];
    Event events[STEP_EVENTS];
} Step;

_Static_assert(sizeof(Config) <= TCP_PAYLOAD, "Config");
_Static_assert(sizeof(Turn) <= TCP_PAYLOAD, "Turn");
_Static_assert(sizeof(Step) <= TCP_PAYLOAD, "Step");

extern volatile sig_atomic_t interrupted;

void turn_make(const World* w, int id, Turn* turn);
void turn_run(World* w, int id, const Turn* turn, Step* step);
bool turn_apply(World* w, int id, const Step* step);
int server_run(World* w, const char* ip, const char* port);
void draw(const World* w, Text* out, bool verbose);
void report(const World* w, Text* out, int code);

#endif
