#include "lb_streamer.h"
#include "lb_streamer_utils.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cctype>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <thread>

#include <yaml-cpp/yaml.h>

std::atomic<bool> g_low_bandwidth_running(true);

void lowBandwidthSignalHandler(int signal)
{
    if (signal == SIGINT) {
        std::cout << "\nReceived SIGINT, stopping..." << std::endl;
        g_low_bandwidth_running = false;
    }
}

namespace {

std::string toLowerCopy(std::string value)
{
    for (char& ch : value) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return value;
}

template <typename T>
void readIfExists(const YAML::Node& node, const std::string& key, T& out)
{
    const YAML::Node child = node[key];
    if (child) {
        out = child.as<T>();
    }
}

CaptureSourceType parseSourceType(const std::string& value, CaptureSourceType fallback)
{
    const std::string lowered = toLowerCopy(value);
    if (lowered == "camera") {
        return CaptureSourceType::LocalCamera;
    }
    if (lowered == "hikrobot") {
        return CaptureSourceType::Hikrobot;
    }
    if (lowered == "file") {
        return CaptureSourceType::File;
    }
    return fallback;
}

std::optional<TransportChannel::Mode> parseTransportMode(const std::string& value)
{
    const std::string lowered = toLowerCopy(value);
    if (lowered == "udp") {
        return TransportChannel::Mode::Udp;
    }
    if (lowered == "serial") {
        return TransportChannel::Mode::Serial;
    }
    return std::nullopt;
}

cv::Rect readRect(const YAML::Node& node, const cv::Rect& fallback)
{
    if (!node || !node.IsMap()) {
        return fallback;
    }

    cv::Rect rect = fallback;
    readIfExists(node, "x", rect.x);
    readIfExists(node, "y", rect.y);
    readIfExists(node, "width", rect.width);
    readIfExists(node, "height", rect.height);
    return rect;
}

bool loadYamlConfig(const std::string& configPath,
                    FrameSource::Config& captureConfig,
                    FramePreprocessor::Config& preprocessConfig,
                    FrameEncoder::Config& encodeConfig,
                    TransportChannel::Config& transportConfig)
{
    if (configPath.empty()) {
        return true;
    }

    YAML::Node root;
    try {
        root = YAML::LoadFile(configPath);
    } catch (const YAML::Exception& e) {
        std::cerr << "Failed to load YAML config '" << configPath << "': " << e.what() << std::endl;
        return false;
    }

    if (root["source"]) {
        captureConfig.sourceType = parseSourceType(root["source"].as<std::string>(), captureConfig.sourceType);
    }
    if (root["codec_name"]) {
        encodeConfig.codecName = root["codec_name"].as<std::string>();
    }
    if (root["video_path"]) {
        captureConfig.videoPath = root["video_path"].as<std::string>();
    }
    if (root["camera_index"]) {
        captureConfig.cameraIndex = root["camera_index"].as<int>();
    }

    const YAML::Node captureNode = root["capture"];
    if (captureNode) {
        readIfExists(captureNode, "input_width", captureConfig.inputWidth);
        readIfExists(captureNode, "input_height", captureConfig.inputHeight);
        readIfExists(captureNode, "capture_fps", captureConfig.captureFps);
        readIfExists(captureNode, "show_preview", captureConfig.showPreview);
        readIfExists(captureNode, "video_path", captureConfig.videoPath);
        readIfExists(captureNode, "camera_index", captureConfig.cameraIndex);
        const YAML::Node sourceNode = captureNode["source_type"];
        if (sourceNode) {
            captureConfig.sourceType = parseSourceType(sourceNode.as<std::string>(), captureConfig.sourceType);
        }
    }

    const YAML::Node hikrobotNode = root["hikrobot"];
    if (hikrobotNode) {
        readIfExists(hikrobotNode, "vid_pid", captureConfig.hikrobotVidPid);
        readIfExists(hikrobotNode, "exposure_ms", captureConfig.hikrobotExposureMs);
        readIfExists(hikrobotNode, "gain", captureConfig.hikrobotGain);
        readIfExists(hikrobotNode, "record_dir", captureConfig.hikrobotRecordDir);
        readIfExists(hikrobotNode, "record_raw_video", captureConfig.hikrobotRecordRawVideo);
        readIfExists(hikrobotNode, "record_fps", captureConfig.hikrobotRecordFps);
    }

    const YAML::Node preprocessNode = root["preprocess"];
    if (preprocessNode) {
        readIfExists(preprocessNode, "enable_roi", preprocessConfig.enableRoi);
        preprocessConfig.roi = readRect(preprocessNode["roi"], preprocessConfig.roi);
        readIfExists(preprocessNode, "output_width", preprocessConfig.outputWidth);
        readIfExists(preprocessNode, "output_height", preprocessConfig.outputHeight);
        readIfExists(preprocessNode, "motion_threshold", preprocessConfig.motionThreshold);
        readIfExists(preprocessNode, "erode_size", preprocessConfig.erodeSize);
        readIfExists(preprocessNode, "dilate_size", preprocessConfig.dilateSize);
        readIfExists(preprocessNode, "blur_kernel", preprocessConfig.blurKernel);
        readIfExists(preprocessNode, "center_protect_size", preprocessConfig.centerProtectSize);
        readIfExists(preprocessNode, "trail_length", preprocessConfig.trailLength);
        readIfExists(preprocessNode, "trail_disable_ratio", preprocessConfig.trailDisableRatio);
    }

    const YAML::Node encodeNode = root["encode"];
    if (encodeNode) {
        readIfExists(encodeNode, "codec_name", encodeConfig.codecName);
        readIfExists(encodeNode, "fps", encodeConfig.fps);
        readIfExists(encodeNode, "bitrate_kbps", encodeConfig.bitrateKbps);
        readIfExists(encodeNode, "gop_size", encodeConfig.gopSize);
        readIfExists(encodeNode, "max_b_frames", encodeConfig.maxBFrames);
        readIfExists(encodeNode, "preset", encodeConfig.preset);
        readIfExists(encodeNode, "tune", encodeConfig.tune);
    }

    const YAML::Node transportNode = root["transport"];
    if (!transportNode) {
        std::cerr << "Missing required transport section in YAML config" << std::endl;
        return false;
    }
    const YAML::Node modeNode = transportNode["mode"];
    if (!modeNode) {
        std::cerr << "Missing required transport.mode in YAML config" << std::endl;
        return false;
    }
    const auto mode = parseTransportMode(modeNode.as<std::string>());
    if (!mode) {
        std::cerr << "Invalid transport.mode. Use 'udp' or 'serial'" << std::endl;
        return false;
    }
    transportConfig.mode = *mode;
    readIfExists(transportNode, "udp_address", transportConfig.udpAddress);
    readIfExists(transportNode, "udp_port", transportConfig.udpPort);
    readIfExists(transportNode, "serial_port", transportConfig.serialPort);
    readIfExists(transportNode, "serial_baud_rate", transportConfig.serialBaudRate);
    readIfExists(transportNode, "wire_packet_size", transportConfig.wirePacketSize);
    readIfExists(transportNode, "max_send_hz", transportConfig.maxSendHz);
    readIfExists(transportNode, "serial_inter_packet_delay_ms", transportConfig.serialInterPacketDelayMs);

    return true;
}

} // namespace

int main()
{
    signal(SIGINT, lowBandwidthSignalHandler);
    const std::string configPath = "config/send_lb.yaml";

    FrameSource::Config captureConfig;
    FramePreprocessor::Config preprocessConfig;
    FrameEncoder::Config encodeConfig;
    TransportChannel::Config transportConfig;

    if (!loadYamlConfig(configPath, captureConfig, preprocessConfig, encodeConfig, transportConfig)) {
        return -1;
    }

    const CaptureSourceType sourceType = captureConfig.sourceType;

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
    if (transportConfig.mode == TransportChannel::Mode::Serial) {
        std::cout << "Transport: Serial(raw) " << transportConfig.serialPort
                  << " @ " << transportConfig.serialBaudRate << std::endl;
    } else {
        std::cout << "Transport: UDP " << transportConfig.udpAddress
                  << ":" << transportConfig.udpPort << std::endl;
    }
    std::cout << "Wire packet: fixed 300 bytes, stream header " << lb_streamer_detail::kStreamHeaderSize
              << " bytes, payload " << lb_streamer_detail::kStreamPayloadSize << " bytes" << std::endl;
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
