#pragma once

#include "platform_compat.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#include <libavutil/error.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}


#define LOW_BANDWIDTH_TRANSPORT_UDP 1
#define LOW_BANDWIDTH_TRANSPORT_SERIAL 2
#define LOW_BANDWIDTH_TRANSPORT_SERIAL_FIXED300 3

#ifndef LOW_BANDWIDTH_TRANSPORT_MODE
#define LOW_BANDWIDTH_TRANSPORT_MODE LOW_BANDWIDTH_TRANSPORT_SERIAL
#endif

struct CaptureConfig {
    int cameraIndex = 0;
    int inputWidth = 1920;
    int inputHeight = 1080;
    int captureFps = 60;
    bool showPreview = true;
};

struct PreprocessConfig {
    bool enableRoi = true;
    cv::Rect roi = {};
    int outputWidth = 400;
    int outputHeight = 400;
    int motionThreshold = 18;
    int erodeSize = 1;
    int dilateSize = 4;
    int blurKernel = 11;
    int centerProtectSize = 120;
    int trailLength = 4;
    double trailDisableRatio = 0.18;
};

struct EncodeConfig {
    std::string codecName = "libx265";
    int fps = 60;
    int bitrateKbps = 80;
    int gopSize = 240;
    int maxBFrames = 2;
    std::string preset = "slow";
    std::string tune;
};

struct TransportConfig {
    std::string udpAddress = "127.0.0.1";
    int udpPort = 3334;
#ifdef _WIN32
    std::string serialPort = "COM14";
#else
    std::string serialPort = "/dev/ttyUSB0";
#endif
    std::uint32_t serialBaudRate = 921600;
    int payloadSize = 256;
    int maxSendHz = 45;
    std::uint32_t serialInterPacketDelayMs = 16;
};

class LowBandwidthStreamer {
public:
    LowBandwidthStreamer(const CaptureConfig& captureConfig = {},
                         const PreprocessConfig& preprocessConfig = {},
                         const EncodeConfig& encodeConfig = {},
                         const TransportConfig& transportConfig = {});
    ~LowBandwidthStreamer();

    bool initialize();
    void startStreaming();
    void stopStreaming();
    bool isStreaming() const { return m_isStreaming.load(); }
    uint64_t totalBytesSent() const { return m_totalBytesSent.load(); }
    uint64_t totalPacketsSent() const { return m_totalPacketsSent.load(); }
    uint64_t totalFramesSent() const { return m_totalFramesSent.load(); }

private:
    struct QueuedFrame {
        cv::Mat frame;
        uint64_t index = 0;
    };

    bool initCamera();
    bool initSocket();
    bool initSerial();
    bool initEncoder();
    void cleanup();

    void captureLoop();
    void processLoop();

    cv::Rect computeEffectiveRoi(const cv::Size& frameSize) const;
    cv::Mat preprocessFrame(const cv::Mat& frame);
    cv::Mat buildMotionMask(const cv::Mat& grayFrame, double& motionRatio);
    void applyStaticSuppression(const cv::Mat& source,
                                const cv::Mat& grayFrame,
                                const cv::Mat& motionMask,
                                cv::Mat& output) const;
    void applyCenterProtection(const cv::Mat& source, cv::Mat& output) const;
    void applyTrail(const cv::Mat& grayFrame,
                    const cv::Mat& motionMask,
                    double motionRatio,
                    cv::Mat& output);

    bool encodeFrame(const cv::Mat& frame, uint32_t frameIndex);
    bool flushEncoder();
    bool sendEncodedPayload(const uint8_t* data, int size, uint32_t frameIndex);
    void logAvError(const std::string& prefix, int errnum) const;

private:
    CaptureConfig m_captureConfig;
    PreprocessConfig m_preprocessConfig;
    EncodeConfig m_encodeConfig;
    TransportConfig m_transportConfig;

    cv::VideoCapture m_cap;
    socket_t m_udpSocket = kInvalidSocket;
    sockaddr_in m_serverAddr = {};
    serial_handle_t m_serialHandle = kInvalidSerialHandle;

    const AVCodec* m_codec = nullptr;
    AVCodecContext* m_codecContext = nullptr;
    AVFrame* m_yuvFrame = nullptr;
    AVPacket* m_packet = nullptr;
    SwsContext* m_swsContext = nullptr;

    std::thread m_captureThread;
    std::thread m_processThread;
    std::mutex m_queueMutex;
    std::condition_variable m_queueCv;
    std::deque<QueuedFrame> m_frameQueue;
    const size_t m_maxQueueSize = 3;

    std::atomic<bool> m_initialized{false};
    std::atomic<bool> m_isStreaming{false};
    std::atomic<bool> m_stopRequested{false};
    std::atomic<uint32_t> m_outputFrameCounter{0};
    std::atomic<uint64_t> m_totalBytesSent{0};
    std::atomic<uint64_t> m_totalPacketsSent{0};
    std::atomic<uint64_t> m_totalFramesSent{0};
    uint64_t m_captureIndex = 0;

    cv::Mat m_previousGray;
    std::deque<cv::Mat> m_trailHistory;
};
