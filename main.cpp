#include "CameraStreamer.h"
#include <iostream>
#include <csignal>
#include "platform_compat.h"

std::atomic<bool> g_running(true);

void signalHandler(int signal) {

    if (signal == SIGINT) {
        std::cout << "\nReceived SIGINT, stopping..." << std::endl;
        g_running = false;
    }
}
int cnt1;
int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    if (argc > 1)
    std::cout << "选用编码器名: " << argv[1] << std::endl;

    CameraStreamer streamer(
        0,
        1920, 1080,
        30,
        "127.0.0.1",
        3334,
        argc > 1 ? argv[1] : "libx265"
    );

    if (!streamer.initialize()) {
        std::cerr << "Failed to initialize CameraStreamer" << std::endl;
        return -1;
    }

    std::cout << "\n=== Camera HEVC Streamer ===" << std::endl;
    std::cout << "Press ESC in video window to stop" << std::endl;
    std::cout << "Press Ctrl+C in console to stop" << std::endl;
    std::cout << "Streaming to UDP port 3334" << std::endl;
    std::cout << "============================\n" << std::endl;

    streamer.startStreaming();
    while (g_running && streamer.isStreaming()) {
        sleep_ms(1000);
        std::cout << "============================\n" << cnt - cnt1 << std::endl;
        cnt1 = cnt;
    }

    streamer.stopStreaming();
    std::cout << "Streaming stopped. Goodbye!" << std::endl;
    return 0;
}

