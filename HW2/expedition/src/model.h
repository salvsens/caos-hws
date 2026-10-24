#ifndef MODEL_H
#define MODEL_H
// =============================================================================
// model.h — общие типы, константы и прототипы многопоточной модели
// «Планетарная исследовательская экспедиция» (ИДЗ №2, вариант 50).
// Всё изменяемое общее состояние мира защищено мьютексом World.mx.
// =============================================================================
#include <pthread.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MAXR    30      // строк карты
#define MAXC    64      // столбцов карты
#define MAXROB  9       // роботов (на карте — цифры 1..9)
#define MAXT    26      // целей (буквы a..z)
#define MAXCH   8       // зарядных станций
#define MAXNODE 9       // узлов связи: антенна базы + до 8 ретрансляторов
#define MAXMEM  16      // память робота, пакетов
#define MAXPKT  4096    // уникальных номеров пакетов
#define NODEQ   256     // сообщений в пути через один узел
#define MBOX    256     // ёмкость ящика базы
#define MAXPART (MAXROB + 1 + MAXNODE + MAXCH)   // потоков-участников такта
#define INF     1000000 // нет пути

typedef struct { int r, c; } Pos;

// Генератор случайных чисел: у каждого потока свой (общий ГСЧ пришлось бы защищать)
typedef struct { uint64_t s; } Rng;

// Приборы роботов — битовая маска
enum { CAM = 1, SPEC = 2, DRILL = 4, RAD = 8, SEIS = 16, MAG = 32 };

typedef enum { T_PENDING, T_ASSIGNED, T_ACTIVE, T_DONE, T_UNAVAIL } TaskState;
typedef enum { R_IDLE, R_TO_TARGET, R_MEASURE, R_TO_COMM, R_UPLOAD, R_TO_CHARGER,
               R_QUEUE, R_CHARGE, R_PARKED, R_FAILED } RobotState;
typedef enum { M_ASSIGN, M_DATA, M_ACK, M_STOP } MsgType;   // M_STOP — команда базе завершиться
typedef enum { END_NONE, END_DONE, END_TICKS, END_EXHAUSTED, END_SIGNAL, END_INVARIANT } EndReason;

// Сообщение канала связи (at — такт доставки, copy — копия, созданная дублированием)
typedef struct { MsgType type; int robot, task, uid, seq, at; bool copy; } Msg;

// Пакет данных в памяти робота (хранится до подтверждения)
typedef struct { int uid, task, seq, last_tx; } Packet;

typedef struct {
    char name[16];
    unsigned instr;          // приборы
    int emax, energy;        // ёмкость батареи и текущий запас
    int mem_cap;             // ёмкость памяти, пакетов
    Pos pos;
    RobotState st;
    int task;                // текущее задание (-1 — нет)
    bool measured;           // данные по заданию уже собраны
    int measure_left;        // осталось тактов исследования
    int goal;                // цель движения: номер задания или станции
    int blocked;             // тактов подряд путь занят
    bool in_comm;            // в зоне связи
    bool yield;              // другой робот просит уступить дорогу
    Packet mem[MAXMEM];
    int nmem;
    Rng rng;
    int moves, sent, retries, charged;   // статистика
} Robot;

typedef struct {
    char id;                 // буква цели
    Pos pos;
    unsigned instr;          // требуемые приборы
    int ticks, volume;       // время исследования, объём данных (пакетов)
    TaskState st;
    int robot;               // исполнитель (-1 — нет)
    int t_assign, last_contact, limit; // такт назначения, последний контакт, допустимое молчание
    int attempts, done_tick;
    unsigned unable;         // роботы, отказавшиеся от задания
    bool chunk[MAXMEM];      // какие фрагменты данных получены базой
    int nchunks;
    char note[128];          // причина недоступности
} Task;

// Узел связи: антенна базы (узел 0) или ретранслятор. Пересылает сообщения,
// которые идут через его зону: задерживает, теряет, дублирует.
typedef struct {
    Pos pos;
    int radius;
    Msg q[NODEQ]; int nq;    // сообщения в пути через узел
    Rng rng;
    int fwd, lost, dup;      // статистика: переслано, потеряно, продублировано
} Node;

typedef struct {
    uint64_t seed;
    int max_ticks, strategy, delay_ms, verbose;   // strategy: 0 fifo, 1 nearest
    bool map_view;
    char log_path[256];
    int move_cost, danger_cost, measure_cost, tx_cost, recharge_pct, charge_rate, charger_stock, charger_slots;
    double danger_fail, p_loss, p_dup;
    int net_delay, net_jitter, ack_timeout, base_radius, relay_radius;
    int assign_timeout, task_timeout, stall_ticks;
    int nrob; char rname[MAXROB][16]; unsigned rinstr[MAXROB]; int remax[MAXROB], rmem[MAXROB];
    int ntask; char tid[MAXT]; unsigned tinstr[MAXT]; int tticks[MAXT], tvol[MAXT];
    int rows; char map[MAXR][MAXC + 2];
} Config;

typedef struct {
    Config cfg;
    pthread_mutex_t mx;                            // защищает всё изменяемое состояние мира
    bool stop;                                     // часы: участникам пора завершаться
    int rows, cols, tick;
    char grid[MAXR][MAXC];
    EndReason end;
    char violation[160];
    Robot rob[MAXROB]; int nrob;
    Task task[MAXT]; int ntask;
    Pos ch[MAXCH]; int chstock[MAXCH]; int nch;   // зарядные станции и их запас (-1 — без ограничения)
    sem_t* slots[MAXCH];                           // свободные разъёмы станций (счётные семафоры)
    Pos base;
    Node node[MAXNODE]; int nnode;                 // узлы связи
    int zone[MAXR][MAXC];                          // узел, в зоне которого клетка (-1 — нет связи)
    int dtask[MAXT][MAXR][MAXC];                   // поля расстояний до целей,
    int dch[MAXCH][MAXR][MAXC];                    // до станций,
    int dcomm[MAXR][MAXC];                         // до зоны связи
    Msg inbox[MBOX]; int in_head, in_count;        // ящик базы: кольцевой буфер пакетов
    sem_t* inbox_sem;                              // число пакетов в ящике базы
    Rng base_rng, disp_rng;                        // ГСЧ базы и диспетчера
    bool got[MAXPKT]; int next_uid, unique, dups;  // база: засчитанные пакеты
    int sent[3], lost[3], dupd[3];                 // статистика канала
    int last_progress, checks;
    int nthreads;                                  // запущено потоков
} World;

// config.c
int  config_load(Config* c, int argc, char* argv[]);
unsigned parse_instr(const char* s);
void format_instr(unsigned m, char* buf, size_t n);
// map.c
int  map_init(World* w, char* err, size_t n);
bool passable(const World* w, Pos p);
bool is_dock(const World* w, Pos p);
int  cell_cost(const World* w, Pos p);
void field(const World* w, const Pos* goals, int ngoals, int d[MAXR][MAXC]);
Pos  step_to(const World* w, int d[MAXR][MAXC], Pos from);
Pos  field_end(const World* w, int d[MAXR][MAXC], Pos from);
// net.c
void net_send(World* w, Rng* g, MsgType t, int robot, int task, int uid, int seq);
void node_tick(World* w, int k);
void* base_thread(void* arg);
void dispatcher_init(World* w);
void dispatcher_tick(World* w);
// robot.c
void robot_tick(World* w, Robot* r);
void robot_on_assign(World* w, Robot* r, int task);
void robot_on_ack(World* w, Robot* r, int uid);
void robot_comm_update(World* w, Robot* r);
bool task_feasible(World* w, Robot* r, int t);
int  reserve(const World* w, Pos p);
const char* robot_state(RobotState s);
// sim.c
void rnd_seed(Rng* g, uint64_t seed, int id);
bool chance(Rng* g, double p);
int  rnd_int(Rng* g, int n);
int  world_init(World* w, char* err, size_t n);
void charger_tick(World* w, int c);
void sim_start(World* w);
void sim_tick(World* w);
void sim_stop(World* w);
EndReason check_end(World* w);
// main.c
void ev(World* w, int level, const char* fmt, ...) __attribute__((format(printf, 3, 4)));

#endif
