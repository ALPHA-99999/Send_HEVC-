#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <netinet/in.h>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

class HikRobotCamera;

using socket_t = int;
using serial_handle_t = int;

inline constexpr socket_t kInvalidSocket = -1;
inline constexpr serial_handle_t kInvalidSerialHandle = -1;

enum class CaptureSourceType {
    File,
    LocalCamera,
    Hikrobot,
};

class TransportChannel {
public:
    enum class Mode {
        Udp,
        Serial,
    };

    struct Config {
        Mode mode = Mode::Udp;
        std::string udpAddress = "192.168.12.2";
        int udpPort = 3334;
        std::string serialPort = "/dev/ttyUSB0";
        std::uint32_t serialBaudRate = 921600;
        int wirePacketSize = 300;
        int maxSendHz = 50;
        std::uint32_t serialInterPacketDelayMs = 20;
    };

    explicit TransportChannel(const Config& config);
    ~TransportChannel();

    bool open();
    void close();
    bool send(const uint8_t* data, size_t size);
    uint64_t totalBytesSent() const { return m_totalBytesSent.load(); }
    uint64_t totalPacketsSent() const { return m_totalPacketsSent.load(); }

private:
    bool openSocket();
    bool openSerial();

    Config m_config;
    int m_udpSocket = kInvalidSocket;
    sockaddr_in m_serverAddr = {};
    int m_serialHandle = kInvalidSerialHandle;
    std::atomic<uint64_t> m_totalBytesSent{0};
    std::atomic<uint64_t> m_totalPacketsSent{0};
};

class FrameSource {
public:
    struct Config {
        CaptureSourceType sourceType = CaptureSourceType::File;
        int cameraIndex = 0;
        std::string videoPath;
        std::string hikrobotVidPid;
        double hikrobotExposureMs = 15.0;
        double hikrobotGain = 14.0;
        std::string hikrobotRecordDir = "/home/arty/Documents/video";
        bool hikrobotRecordRawVideo = false;
        double hikrobotRecordFps = 60.0;
        int inputWidth = 1920;
        int inputHeight = 1080;
        int captureFps = 60;
        bool showPreview = true;
    };

    explicit FrameSource(const Config& config);
    ~FrameSource();

    bool open();
    void close();
    bool read(cv::Mat& frame, std::chrono::steady_clock::time_point& timestamp);
    bool reopenCamera(int index);

    CaptureSourceType sourceType() const { return m_config.sourceType; }
    int cameraIndex() const { return m_cameraIndex; }
    const Config& config() const { return m_config; }
    Config& config() { return m_config; }

private:
    Config m_config;
    cv::VideoCapture m_cap;
    std::unique_ptr<HikRobotCamera> m_hikrobotCamera;
    int m_cameraIndex = 0;
    bool m_opened = false;
};

class FramePreprocessor {
public:
    struct Config {
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

    explicit FramePreprocessor(const Config& config);

    void reset();
    cv::Rect computeEffectiveRoi(const cv::Size& frameSize) const;
    cv::Rect currentRuntimeRoi(const cv::Size& frameSize);
    void moveRuntimeRoi(const cv::Size& frameSize, int dx, int dy);
    cv::Mat process(const cv::Mat& frame, const cv::Rect& roi);
    cv::Mat makePreviewFrame(const cv::Mat& originalFrame,
                             const cv::Mat& processedFrame,
                             const cv::Rect& roi) const;

private:
    cv::Rect clampRoiToFrame(const cv::Rect& roi, const cv::Size& frameSize) const;
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

private:
    Config m_config;
    cv::Rect m_runtimeRoi;
    bool m_runtimeRoiInitialized = false;
    cv::Mat m_previousGray;
    std::deque<cv::Mat> m_trailHistory;
};

class WirePacketizer {
public:
    explicit WirePacketizer(const TransportChannel::Config& config);

    void reset();
    bool pushEncodedPayload(const uint8_t* data,
                            int size,
                            bool keyframe,
                            const std::function<bool(const uint8_t* packet, size_t packetSize)>& sendPacket);
    bool flush(bool finalFlush,
               const std::function<bool(const uint8_t* packet, size_t packetSize)>& sendPacket);

private:
    bool sendWirePacket(const uint8_t* payload,
                        size_t payloadLen,
                        uint8_t flags,
                        uint16_t syncOffset,
                        const std::function<bool(const uint8_t* packet, size_t packetSize)>& sendPacket);

private:
    struct EncodedChunk {
        std::vector<uint8_t> data;
        bool keyframe = false;
    };

    TransportChannel::Config m_config;
    std::deque<EncodedChunk> m_encodedChunks;
    std::size_t m_encodedChunkOffset = 0;
    uint32_t m_nextPacketSeq = 0;
};

class FrameEncoder {
public:
    struct Config {
        std::string codecName = "hevc_vaapi";
        int fps = 60;
        int bitrateKbps = 80;
        int gopSize = 240;
        int maxBFrames = 2;
        std::string preset = "slow";
        std::string tune;
    };

    FrameEncoder(const Config& config,
                 const FramePreprocessor::Config& preprocessConfig,
                 const TransportChannel::Config& transportConfig);
    ~FrameEncoder();

    void setSendPacketCallback(std::function<bool(const uint8_t* packet, size_t packetSize)> sendPacket);
    bool open();
    void close();
    bool encodeFrame(const cv::Mat& frame, uint32_t frameIndex);
    bool flush();
    void resetStats();
    uint64_t totalEncodeMicros() const { return m_totalEncodeMicros.load(); }

private:
    bool initHardwareEncoder();
    void logAvError(const std::string& prefix, int errnum) const;

private:
    Config m_config;
    FramePreprocessor::Config m_preprocessConfig;
    TransportChannel::Config m_transportConfig;
    WirePacketizer m_packetizer;

    const AVCodec* m_codec = nullptr;
    AVCodecContext* m_codecContext = nullptr;
    AVFrame* m_yuvFrame = nullptr;
    AVFrame* m_encodeSwFrame = nullptr;
    AVFrame* m_encodeHwFrame = nullptr;
    AVPacket* m_packet = nullptr;
    SwsContext* m_swsContext = nullptr;
    AVBufferRef* m_encodeHwDeviceCtx = nullptr;
    AVBufferRef* m_encodeHwFramesCtx = nullptr;
    bool m_useHardwareEncoder = false;
    AVPixelFormat m_encoderInputPixelFormat = AV_PIX_FMT_YUV420P;
    std::function<bool(const uint8_t* packet, size_t packetSize)> m_sendPacket;
    std::atomic<uint64_t> m_totalEncodeMicros{0};
};

class WireDecoder {
public:
    WireDecoder() = default;

    bool open();
    void close();
};
