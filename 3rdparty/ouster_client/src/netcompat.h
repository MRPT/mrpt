/**
 * Copyright (c) 2020, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Compatibility with windows (unsupported)
 */

#pragma once

#include <string>

#if defined _WIN32  // --------- On Windows ---------

// Try and limit the stuff windows brings in
#include <winsock2.h>
#include <ws2tcpip.h>

// ssize_t is available in vs
#ifdef _MSC_VER
#include <BaseTsd.h>
#define ssize_t SSIZE_T
#endif

#else  // --------- Compiling on *nix ---------

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>

// Define windows types
#ifndef SOCKET
#define SOCKET int
#endif

#ifndef FDSET
#define FDSET fd_set
#endif

#define SOCKET_ERROR -1

#endif  // --------- End Platform Differentiation Block ---------

namespace ouster {
namespace sensor {
namespace impl {

/**
 * Close a specified socket
 * @param[in] sock The socket file descriptor to close
 * @return success
 */
int socket_close(SOCKET sock);

/**
 * Get the error message for socket errors
 * @return The socket error message
 */
std::string socket_get_error();

/**
 * Check if a socket file descriptor is valid
 * @param[in] value The socket file descriptor to check
 * @return The validity of the socket file descriptor
 */
bool socket_valid(SOCKET value);

/**
 * Check if the last error was a socket exit event
 * @return If the socket has exited
 */
bool socket_exit();

/**
 * Set a specified socket to non-blocking
 * @param[in] value The socket file descriptor to set non-blocking
 * @return success
 */
int socket_set_non_blocking(SOCKET value);

/**
 * Set a specified socket to reuse
 * @param[in] value The socket file descriptor to set reuse
 * @return success
 */
int socket_set_reuse(SOCKET value);

/**
 * Set SO_RCVTIMEO on the specified socket
 * @param[in] sock The socket file descriptor
 * @param[in] timeout_sec Timeout esconds
 * @return success
 */
int socket_set_rcvtimeout(SOCKET sock, int timeout_sec);

}  // namespace impl
}  // namespace sensor
}  // namespace ouster
