#pragma once

#include <cstdint>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#else
#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <netinet/in.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#endif

using socket_t =
#ifdef _WIN32
    SOCKET;
#else
    int;
#endif

using serial_handle_t =
#ifdef _WIN32
    HANDLE;
#else
    int;
#endif

inline constexpr socket_t kInvalidSocket =
#ifdef _WIN32
    INVALID_SOCKET;
#else
    -1;
#endif

inline constexpr serial_handle_t kInvalidSerialHandle =
#ifdef _WIN32
    INVALID_HANDLE_VALUE;
#else
    -1;
#endif

inline bool socket_init() {
#ifdef _WIN32
    WSADATA wsaData;
    return WSAStartup(MAKEWORD(2, 2), &wsaData) == 0;
#else
    return true;
#endif
}

inline void socket_cleanup() {
#ifdef _WIN32
    WSACleanup();
#endif
}

inline int socket_last_error() {
#ifdef _WIN32
    return WSAGetLastError();
#else
    return errno;
#endif
}

inline int close_socket(socket_t sock) {
#ifdef _WIN32
    return closesocket(sock);
#else
    return close(sock);
#endif
}

inline void sleep_ms(std::uint32_t ms) {
#ifdef _WIN32
    Sleep(ms);
#else
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
#endif
}
