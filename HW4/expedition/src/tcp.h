#ifndef TCP_H
#define TCP_H
// =============================================================================
// tcp.h — открытие TCP-сокетов сервера и клиента по адресу из командной строки.
// =============================================================================

// Возвращают открытый дескриптор или -1 с установленным errno.
int tcp_listen(const char* ip, const char* port);
#include <stddef.h>
#include <stdint.h>

int tcp_connect(const char* ip, const char* port);
int tcp_nonblock(int fd);

#define TCP_PAYLOAD 16384
#define TCP_HEADER (2 * sizeof(uint32_t))

typedef struct {
    uint32_t header[2];                         // заголовок на проводе
    size_t used;                                // байтов получено в текущем кадре
    uint32_t type, len;                         // тип и длина последнего полного кадра
    unsigned char data[TCP_PAYLOAD];
} Frame;

int tcp_read(int fd, Frame* frame);              // 1 — кадр, 0 — ещё ждать, -1 — обрыв/ошибка
size_t tcp_pack(void* dst, uint32_t type, const void* data, size_t len);
int tcp_send(int fd, uint32_t type, const void* data, size_t len);

#endif
