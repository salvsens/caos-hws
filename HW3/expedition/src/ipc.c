// =============================================================================
// ipc.c — объекты взаимодействия процессов: разделяемая память POSIX с миром
// модели и семафоры POSIX. Два варианта сборки:
//   по умолчанию — именованные семафоры (sem_open, имена «key_номер»);
//   -DUNNAMED   — безымянные семафоры в разделяемой памяти (sem_init, pshared = 1).
// Остальной код обращается к семафорам только по номеру (S_MX, S_GO + i, ...).
// =============================================================================
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include "model.h"

static sem_t* g_sem[NSEM];   // семафоры этого процесса (наследуются при fork)

#ifndef UNNAMED
static void sem_name(const char* key, int k, char* buf, size_t n) { snprintf(buf, n, "%s_%d", key, k); }
#endif

// Удалить объекты с этим именем (остались от аварийно завершённого запуска)
static void unlink_all(const char* key) {
    shm_unlink(key);
#ifndef UNNAMED
    char name[48];
    for (int k = 0; k < NSEM; k++) {
        sem_name(key, k, name, sizeof name);
        sem_unlink(name);
    }
#endif
}

// Часы: создать память, скопировать в неё готовый мир, создать семафоры
World* ipc_create(const World* init, const char* key) {
    unlink_all(key);
    int fd = shm_open(key, O_CREAT | O_EXCL | O_RDWR, 0600);
    if (fd < 0) { dprintf(2, "shm_open %s: %s\n", key, strerror(errno)); return NULL; }
    if (ftruncate(fd, sizeof(World)) != 0) {
        dprintf(2, "ftruncate: %s\n", strerror(errno));
        close(fd); shm_unlink(key);
        return NULL;
    }
    World* w = mmap(NULL, sizeof(World), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);   // отображение остаётся и без дескриптора
    if (w == MAP_FAILED) { dprintf(2, "mmap: %s\n", strerror(errno)); shm_unlink(key); return NULL; }
    memcpy(w, init, sizeof(World));
    snprintf(w->key, sizeof w->key, "%s", key);
    for (int k = 0; k < NSEM; k++) {   // разъёмы станций — charger_slots, S_MX — 1, остальные — 0
        unsigned v = k == S_MX ? 1 : k >= S_SLOT ? (unsigned)w->cfg.charger_slots : 0;
#ifdef UNNAMED
        if (sem_init(&w->sem[k], 1, v) != 0) {
            dprintf(2, "sem_init: %s — безымянные семафоры между процессами не поддерживаются, "
                       "нужен Linux\n", strerror(errno));
            ipc_destroy(w);
            return NULL;
        }
        g_sem[k] = &w->sem[k];
#else
        char name[48];
        sem_name(key, k, name, sizeof name);
        if ((g_sem[k] = sem_open(name, O_CREAT | O_EXCL, 0600, v)) == SEM_FAILED) {
            dprintf(2, "sem_open %s: %s\n", name, strerror(errno));
            g_sem[k] = NULL;
            ipc_destroy(w);
            return NULL;
        }
#endif
    }
    w->ready = true;   // теперь участники могут подключаться
    return w;
}

// Независимый участник: подключиться к миру, созданному часами
World* ipc_attach(const char* key) {
    int fd = shm_open(key, O_RDWR, 0);
    if (fd < 0) {
        dprintf(2, "Мир %s не найден: сначала запустите часы (./expedition -R clock)\n", key);
        return NULL;
    }
    struct stat st;
    if (fstat(fd, &st) != 0 || st.st_size < (off_t)sizeof(World)) {
        dprintf(2, "Мир %s ещё не создан часами, повторите запуск\n", key);
        close(fd);
        return NULL;
    }
    World* w = mmap(NULL, sizeof(World), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (w == MAP_FAILED) { dprintf(2, "mmap: %s\n", strerror(errno)); return NULL; }
    if (!w->ready) {
        dprintf(2, "Мир %s ещё не создан часами, повторите запуск\n", key);
        munmap(w, sizeof(World));
        return NULL;
    }
    for (int k = 0; k < NSEM; k++) {
#ifdef UNNAMED
        g_sem[k] = &w->sem[k];
#else
        char name[48];
        sem_name(key, k, name, sizeof name);
        if ((g_sem[k] = sem_open(name, 0)) == SEM_FAILED) {
            dprintf(2, "sem_open %s: %s (часы ещё не готовы?)\n", name, strerror(errno));
            g_sem[k] = NULL;
            ipc_detach(w);
            return NULL;
        }
#endif
    }
    return w;
}

// Участник: закрыть свои семафоры и отображение (объекты удаляют часы)
void ipc_detach(World* w) {
#ifndef UNNAMED
    for (int k = 0; k < NSEM; k++)
        if (g_sem[k]) { sem_close(g_sem[k]); g_sem[k] = NULL; }
#endif
    munmap(w, sizeof(World));
}

// Часы в конце работы: удалить все семафоры и разделяемую память
void ipc_destroy(World* w) {
    char key[32];
    snprintf(key, sizeof key, "%s", w->key);
#ifdef UNNAMED
    for (int k = 0; k < NSEM; k++)
        if (g_sem[k]) { sem_destroy(g_sem[k]); g_sem[k] = NULL; }
#endif
    ipc_detach(w);
    unlink_all(key);
}

// Ожидание семафора; прерывание сигналом (EINTR) — ждать дальше
void ipc_wait(int k) { while (sem_wait(g_sem[k]) != 0) {} }
// Одна попытка ожидания: false — прервано сигналом
bool ipc_wait_once(int k) { return sem_wait(g_sem[k]) == 0; }
void ipc_post(int k) { sem_post(g_sem[k]); }
bool ipc_trywait(int k) { return sem_trywait(g_sem[k]) == 0; }
void ipc_lock(void) { ipc_wait(S_MX); }
void ipc_unlock(void) { sem_post(g_sem[S_MX]); }
