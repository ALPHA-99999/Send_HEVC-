#include "BufferedLowBandwidthStreamer.h"

#include <atomic>
#include <csignal>
#include "platform_compat.h"
#include <iomanip>
#include <iostream>
#include <string>

std::atomic<bool> g_buffered_low_bandwidth_running(true);

void bufferedLowBandwidthSignalHandler(int signal)
{
    if (signal == SIGINT) {
        std::cout << "\nReceived SIGINT, stopping..." << std::endl;
        g_buffered_low_bandwidth_running = false;
    }
}

int main(int argc, char* argv[])
{
    signal(SIGINT, bufferedLowBandwidthSignalHandler);

    std::string codecName = "libx265";
    if (argc > 1) {
        codecName = argv[1];
        std::cout << "Selected codec: " << codecName << std::endl;
    }

    CaptureConfig captureConfig;
    captureConfig.cameraIndex = 0;
    captureConfig.inputWidth = 1920;
    captureConfig.inputHeight = 1080;
    captureConfig.captureFps = 30;

    PreprocessConfig preprocessConfig;
    preprocessConfig.outputWidth = 300;
    preprocessConfig.outputHeight = 300;
    preprocessConfig.centerProtectSize = 200;
    preprocessConfig.trailLength = 5;

    EncodeConfig encodeConfig;
    encodeConfig.codecName = codecName;
    encodeConfig.fps = 30;
    encodeConfig.bitrateKbps = 90;
    encodeConfig.gopSize = 120;
    encodeConfig.maxBFrames = 2;
    encodeConfig.tune.clear();

    TransportConfig transportConfig;
    transportConfig.udpAddress = "127.0.0.1";
    transportConfig.udpPort = 3334;
#ifdef _WIN32
    transportConfig.serialPort = "COM11";
#else
    transportConfig.serialPort = "/dev/ttyUSB0";
#endif
    transportConfig.serialBaudRate = 921600;
    transportConfig.payloadSize = 292;
    transportConfig.maxSendHz = 65;
    transportConfig.serialInterPacketDelayMs = 0;
    BufferedLowBandwidthStreamer streamer(
        captureConfig, preprocessConfig, encodeConfig, transportConfig);
    if (!streamer.initialize()) {
        std::cerr << "Failed to initialize BufferedLowBandwidthStreamer" << std::endl;
        return -1;
    }

    std::cout << "\n=== Buffered Low Bandwidth HEVC Streamer ===" << std::endl;
    std::cout << "Input source: OpenCV cap(0)" << std::endl;
#if LOW_BANDWIDTH_TRANSPORT_MODE == LOW_BANDWIDTH_TRANSPORT_SERIAL
    std::cout << "Transport: Serial(raw) " << transportConfig.serialPort
              << " @ " << transportConfig.serialBaudRate << std::endl;
#elif LOW_BANDWIDTH_TRANSPORT_MODE == LOW_BANDWIDTH_TRANSPORT_SERIAL_FIXED300
    std::cout << "Transport: Serial(fixed300+crc) " << transportConfig.serialPort
              << " @ " << transportConfig.serialBaudRate << std::endl;
#else
    std::cout << "Transport: UDP " << transportConfig.udpAddress
              << ":" << transportConfig.udpPort << std::endl;
#endif
    std::cout << "Policy: encode continuously, rate-limit on send, drop oldest encoded backlog"
              << std::endl;
    std::cout << "Press ESC in video window to stop" << std::endl;
    std::cout << "Press Ctrl+C in console to stop" << std::endl;
    std::cout << "===========================================\n" << std::endl;

    streamer.startStreaming();
    uint64_t lastBytesSent = 0;
    uint64_t lastPacketsSent = 0;
    uint64_t lastFramesSent = 0;
    uint64_t lastDroppedFrames = 0;
    while (g_buffered_low_bandwidth_running && streamer.isStreaming()) {
        sleep_ms(1000);
        const uint64_t bytesSent = streamer.totalBytesSent();
        const uint64_t packetsSent = streamer.totalPacketsSent();
        const uint64_t framesSent = streamer.totalFramesSent();
        const uint64_t droppedFrames = streamer.droppedFrames();
        const uint64_t packetsThisSecond = packetsSent - lastPacketsSent;
        const uint64_t framesThisSecond = framesSent - lastFramesSent;
        const double avgPacketsPerFrame =
            framesThisSecond > 0
                ? static_cast<double>(packetsThisSecond) / static_cast<double>(framesThisSecond)
                : 0.0;

        std::cout << std::fixed << std::setprecision(2)
                  << "TX " << (static_cast<double>(bytesSent - lastBytesSent) / 1024.0) << " KB/s"
                  << " | packets/s " << packetsThisSecond
                  << " | frames/s " << framesThisSecond
                  << " | avg packets/frame " << avgPacketsPerFrame
                  << " | dropped/s " << (droppedFrames - lastDroppedFrames)
                  << std::endl;

        lastBytesSent = bytesSent;
        lastPacketsSent = packetsSent;
        lastFramesSent = framesSent;
        lastDroppedFrames = droppedFrames;
    }

    streamer.stopStreaming();
    std::cout << "Streaming stopped. Goodbye!" << std::endl;
    return 0;
}
