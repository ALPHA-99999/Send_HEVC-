#include "lb_streamer.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <thread>

std::atomic<bool> g_low_bandwidth_running(true);

void lowBandwidthSignalHandler(int signal)
{
    if (signal == SIGINT) {
        std::cout << "\nReceived SIGINT, stopping..." << std::endl;
        g_low_bandwidth_running = false;
    }
}

static std::string get_arg(int argc, char ** argv, const std::string & key, const std::string & default_value)
{
    const std::string prefix = key + "=";
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg.rfind(prefix, 0) == 0) {
            return arg.substr(prefix.size());
        }
    }
    return default_value;
}

int main(int argc, char* argv[])
{
    signal(SIGINT, lowBandwidthSignalHandler);

    std::string codecName = "hevc_vaapi";
    CaptureSourceType sourceType = CaptureSourceType::File;
    std::string videoPath = "/home/arty/Documents/video/1.avi";
    std::string hikrobotVidPid;
    double hikrobotExposureMs = 15.0;
    double hikrobotGain = 14.0;
    int cameraIndex = 0;
    std::vector<std::string> positional;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (!arg.empty() && arg[0] != '-') {
            positional.push_back(arg);
        }
    }

    const std::string sourceArg = get_arg(argc, argv, "--source", "");
    if (sourceArg == "camera") {
        sourceType = CaptureSourceType::LocalCamera;
    } else if (sourceArg == "hikrobot") {
        sourceType = CaptureSourceType::Hikrobot;
    } else if (sourceArg == "file") {
        sourceType = CaptureSourceType::File;
    } else if (!positional.empty()) {
        const std::string& arg1 = positional.front();
        if (arg1 == "camera") {
            sourceType = CaptureSourceType::LocalCamera;
        } else if (arg1 == "hikrobot") {
            sourceType = CaptureSourceType::Hikrobot;
        } else if (arg1 == "file") {
            sourceType = CaptureSourceType::File;
        } else {
            codecName = arg1;
            std::cout << "Selected codec: " << codecName << std::endl;
        }
    }

    if (sourceType == CaptureSourceType::LocalCamera) {
        if (positional.size() > 1) {
            cameraIndex = std::stoi(positional[1]);
        }
        if (positional.size() > 2) {
            codecName = positional[2];
            std::cout << "Selected codec: " << codecName << std::endl;
        }
    } else if (sourceType == CaptureSourceType::File) {
        if (positional.size() > 1) {
            videoPath = positional[1];
        }
        if (positional.size() > 2) {
            codecName = positional[2];
            std::cout << "Selected codec: " << codecName << std::endl;
        }
    } else if (sourceType == CaptureSourceType::Hikrobot) {
        if (positional.size() > 1) {
            codecName = positional[1];
            std::cout << "Selected codec: " << codecName << std::endl;
        }
        hikrobotVidPid = get_arg(argc, argv, "--vid-pid", "");
        hikrobotExposureMs = std::stod(get_arg(argc, argv, "--exposure", "15.0"));
        hikrobotGain = std::stod(get_arg(argc, argv, "--gain", "14.0"));
    }

    CaptureConfig captureConfig;
    captureConfig.sourceType = sourceType;
    if (sourceType == CaptureSourceType::LocalCamera) {
        captureConfig.cameraIndex = cameraIndex;
        captureConfig.videoPath.clear();
        captureConfig.inputWidth = 1920;
        captureConfig.inputHeight = 1080;
    } else if (sourceType == CaptureSourceType::Hikrobot) {
        captureConfig.hikrobotVidPid = hikrobotVidPid;
        captureConfig.hikrobotExposureMs = hikrobotExposureMs;
        captureConfig.hikrobotGain = hikrobotGain;
        captureConfig.hikrobotRecordFps = 60.0;
        captureConfig.hikrobotRecordRawVideo = false;
        captureConfig.videoPath.clear();
    } else {
        captureConfig.videoPath = videoPath;
        captureConfig.inputWidth = 1920;
        captureConfig.inputHeight = 1080;
    }
    captureConfig.captureFps = 27;

    PreprocessConfig preprocessConfig;
    preprocessConfig.roi = cv::Rect(560, 140, 800, 800);
    preprocessConfig.outputWidth = 400;
    preprocessConfig.outputHeight = 400;
    preprocessConfig.centerProtectSize = 200;
    preprocessConfig.trailLength = 5;

    EncodeConfig encodeConfig;
    encodeConfig.codecName = codecName;
    encodeConfig.fps = 30;
    encodeConfig.bitrateKbps = 100;
    encodeConfig.gopSize = 120;
    encodeConfig.maxBFrames = 2;
    encodeConfig.tune.clear();

    TransportConfig transportConfig;
    transportConfig.udpAddress = "127.0.0.1";
    transportConfig.udpPort = 3334;
    transportConfig.serialPort = "/dev/ttyUSB0";
    transportConfig.serialBaudRate = 921600;
    transportConfig.payloadSize = 292;
    transportConfig.maxSendHz = 80;
    transportConfig.serialInterPacketDelayMs = 20;

    LowBandwidthStreamer streamer(captureConfig, preprocessConfig, encodeConfig, transportConfig);
    if (!streamer.initialize()) {
        std::cerr << "Failed to initialize LowBandwidthStreamer" << std::endl;
        return -1;
    }

    std::cout << "\n=== Low Bandwidth HEVC Streamer ===" << std::endl;
    if (sourceType == CaptureSourceType::LocalCamera) {
        std::cout << "Input source: local camera cap(" << captureConfig.cameraIndex << ")" << std::endl;
    } else if (sourceType == CaptureSourceType::Hikrobot) {
        std::cout << "Input source: Hikrobot camera";
        if (!captureConfig.hikrobotVidPid.empty()) {
            std::cout << " vid_pid=" << captureConfig.hikrobotVidPid;
        }
        std::cout << std::endl;
    } else {
        std::cout << "Input source: " << captureConfig.videoPath << std::endl;
    }
#if LOW_BANDWIDTH_TRANSPORT_MODE == LOW_BANDWIDTH_TRANSPORT_SERIAL
    std::cout << "Transport: Serial(raw) " << transportConfig.serialPort
              << " @ " << transportConfig.serialBaudRate << std::endl;
#else
    std::cout << "Transport: UDP " << transportConfig.udpAddress
              << ":" << transportConfig.udpPort << std::endl;
#endif
    std::cout << "Press ESC in video window to stop" << std::endl;
    std::cout << "Press Ctrl+C in console to stop" << std::endl;
    std::cout << "============================\n" << std::endl;

    streamer.startStreaming();
    uint64_t lastBytesSent = 0;
    uint64_t lastPacketsSent = 0;
    uint64_t lastFramesSent = 0;
    uint64_t lastEncodeMicros = 0;
    while (g_low_bandwidth_running && streamer.isStreaming()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        const uint64_t bytesSent = streamer.totalBytesSent();
        const uint64_t packetsSent = streamer.totalPacketsSent();
        const uint64_t framesSent = streamer.totalFramesSent();
        const uint64_t encodeMicros = streamer.totalEncodeMicros();
        const uint64_t deltaBytes = bytesSent - lastBytesSent;
        const uint64_t deltaPackets = packetsSent - lastPacketsSent;
        const uint64_t deltaFrames = framesSent - lastFramesSent;
        const uint64_t deltaEncodeMicros = encodeMicros - lastEncodeMicros;
        const double avgEncodeMs = deltaFrames > 0
            ? static_cast<double>(deltaEncodeMicros) / static_cast<double>(deltaFrames) / 1000.0
            : 0.0;

        std::cout << std::fixed << std::setprecision(2)
                  << "TX " << (static_cast<double>(deltaBytes) / 1024.0) << " KB/s"
                  << " | packets/s " << deltaPackets
                  << " | frames/s " << deltaFrames
                  << " | encode ms/frame " << avgEncodeMs
                  << std::endl;

        lastBytesSent = bytesSent;
        lastPacketsSent = packetsSent;
        lastFramesSent = framesSent;
        lastEncodeMicros = encodeMicros;
    }

    streamer.stopStreaming();
    std::cout << "Streaming stopped. Goodbye!" << std::endl;
    return 0;
}
