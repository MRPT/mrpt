/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 */

#include "netcompat.h"

#include <string>

#if defined _WIN32

#include <winsock2.h>

#else

#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

#endif

namespace ouster {
namespace sensor {
namespace impl {

#ifdef _WIN32
struct StaticWrapper {
    WSADATA wsa_data;

    StaticWrapper() { WSAStartup(MAKEWORD(1, 1), &wsa_data); }

    ~StaticWrapper() { WSACleanup(); }
};

static StaticWrapper resources = {};
#endif

int socket_close(SOCKET sock) {
#ifdef _WIN32
    return closesocket(sock);
#else
    return close(sock);
#endif
}

std::string socket_get_error() {
#ifdef _WIN32
    int errnum = WSAGetLastError();
    char buf[256] = {0};
    if (FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM, NULL, errnum, 0, buf,
                      sizeof(buf), NULL) != 0) {
        return std::string(buf);
    } else {
        return std::string{"Unknown WSA error "} + std::to_string(errnum);
    }
#else
    return std::strerror(errno);
#endif
}

bool socket_valid(SOCKET sock) {
#ifdef _WIN32
    return sock != SOCKET_ERROR;
#else
    return sock >= 0;
#endif
}

bool socket_exit() {
#ifdef _WIN32
    auto result = WSAGetLastError();
    return result == WSAECONNRESET || result == WSAECONNABORTED ||
           result == WSAESHUTDOWN;
#else
    return errno == EINTR;
#endif
}

int socket_set_non_blocking(SOCKET value) {
#ifdef _WIN32
    u_long non_blocking_mode = 1;
    return ioctlsocket(value, FIONBIO, &non_blocking_mode);
#else
    return fcntl(value, F_SETFL, fcntl(value, F_GETFL, 0) | O_NONBLOCK);
#endif
}

int socket_set_reuse(SOCKET value) {
    int option = 1;
#ifndef _WIN32
    int res =
        setsockopt(value, SOL_SOCKET, SO_REUSEPORT, &option, sizeof(option));
    if (res != 0) return res;
#endif
    return setsockopt(value, SOL_SOCKET, SO_REUSEADDR, (char*)&option,
                      sizeof(option));
}

int socket_set_rcvtimeout(SOCKET sock, int timeout_sec) {
#ifdef _WIN32
    DWORD timeout_ms = timeout_sec * 1000;
    return setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout_ms,
                      sizeof timeout_ms);
#else
    struct timeval tv;
    tv.tv_sec = timeout_sec;
    tv.tv_usec = 0;
    return setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv,
                      sizeof tv);
#endif
}

}  // namespace impl
}  // namespace sensor
}  // namespace ouster
