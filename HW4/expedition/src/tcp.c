// =============================================================================
// tcp.c — TCP-сокеты IPv4 и передача кадров с заголовком «тип, длина».
// =============================================================================
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include "tcp.h"

static int parse_address(const char* ip, const char* port, struct sockaddr_in* addr) {
    *addr = (struct sockaddr_in){.sin_family = AF_INET};
    char* end;
    errno = 0;
    long number = strtol(port, &end, 10);
    if (*port < '0' || *port > '9' || *end || errno || number < 1 || number > 65535 ||
        inet_pton(AF_INET, ip, &addr->sin_addr) != 1) {
        errno = EINVAL;
        return -1;
    }
    addr->sin_port = htons((uint16_t)number);
    return 0;
}

static int close_error(int fd) {
    int saved = errno;
    close(fd);
    errno = saved;
    return -1;
}

int tcp_listen(const char* ip, const char* port) {
    struct sockaddr_in addr;
    if (parse_address(ip, port, &addr) < 0) return -1;
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    int reuse = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof reuse) < 0 ||
        bind(fd, (const struct sockaddr*)&addr, sizeof addr) < 0 ||
        listen(fd, SOMAXCONN) < 0) return close_error(fd);
    return fd;
}

int tcp_connect(const char* ip, const char* port) {
    struct sockaddr_in addr;
    if (parse_address(ip, port, &addr) < 0) return -1;
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    if (connect(fd, (const struct sockaddr*)&addr, sizeof addr) < 0) return close_error(fd);
    return fd;
}

int tcp_nonblock(int fd) {
    int flags = fcntl(fd, F_GETFL);
    return flags < 0 ? -1 : fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

int tcp_read(int fd, Frame* f) {
    for (;;) {
        void* dst;
        size_t left;
        if (f->used < TCP_HEADER) {
            dst = (unsigned char*)f->header + f->used;
            left = TCP_HEADER - f->used;
        } else {
            uint32_t len = ntohl(f->header[1]);
            if (len > sizeof f->data) { errno = EMSGSIZE; return -1; }
            size_t body = f->used - TCP_HEADER;
            if (body == len) {
                f->type = ntohl(f->header[0]);
                f->len = len;
                f->used = 0;
                return 1;
            }
            dst = f->data + body;
            left = len - body;
        }
        ssize_t n = read(fd, dst, left);
        if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) return 0;
        if (n <= 0) return -1;
        f->used += (size_t)n;
    }
}

size_t tcp_pack(void* dst, uint32_t type, const void* data, size_t len) {
    uint32_t header[] = {htonl(type), htonl((uint32_t)len)};
    memcpy(dst, header, sizeof header);
    if (len) memcpy((unsigned char*)dst + sizeof header, data, len);
    return sizeof header + len;
}

int tcp_send(int fd, uint32_t type, const void* data, size_t len) {
    unsigned char buf[TCP_HEADER + TCP_PAYLOAD];
    if (len > TCP_PAYLOAD) { errno = EMSGSIZE; return -1; }
    size_t size = tcp_pack(buf, type, data, len);
    for (size_t off = 0; off < size;) {
        ssize_t n = write(fd, buf + off, size - off);
        if (n <= 0) return -1;
        off += (size_t)n;
    }
    return 0;
}
