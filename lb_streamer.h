#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <netinet/in.h>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <sys/socket.h>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

using socket_t = int;
using serial_handle_t = int;

inline constexpr socket_t kInvalidSocket = -1;
inline constexpr serial_handle_t kInvalidSerialHandle = -1;

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#include <libavutil/error.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}


#define LOW_BANDWIDTH_TRANSPORT_UDP 1
#define LOW_BANDWIDTH_TRANSPORT_SERIAL 2

#ifndef LOW_BANDWIDTH_TRANSPORT_MODE
#define LOW_BANDWIDTH_TRANSPORT_MODE LOW_BANDWIDTH_TRANSPORT_UDP
#endif

enum class CaptureSourceType {
    File,
    LocalCamera,
    Hikrobot,
};

#ifdef HAVE_HIKROBOT_SDK
class HikRobotCamera;
#endif

struct CaptureConfig {
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
    std::string codecName = "hevc_vaapi";
    int fps = 60;
    int bitrateKbps = 80;
    int gopSize = 240;
    int maxBFrames = 2;
    std::string preset = "slow";
    std::string tune;
};

struct TransportConfig {
    std::string udpAddress = "192.168.12.2";
    int udpPort = 3334;
    std::string serialPort = "/dev/ttyUSB0";
    std::uint32_t serialBaudRate = 921600;
    int payloadSize = 256;
    int maxSendHz = 50;
    std::uint32_t serialInterPacketDelayMs = 20;
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
    uint64_t totalEncodeMicros() const { return m_totalEncodeMicros.load(); }

private:
    struct QueuedFrame {
        cv::Mat frame;
        uint64_t index = 0;
    };

    bool initCamera();
    bool initSocket();
    bool initSerial();
    bool initEncoder();
    bool initDecoder();
    bool initHardwareEncoder();
    void cleanup();

    void captureLoop();
    void processLoop();

    cv::Rect computeEffectiveRoi(const cv::Size& frameSize) const;
    cv::Rect currentRuntimeRoi(const cv::Size& frameSize);
    void moveRuntimeRoi(const cv::Size& frameSize, int dx, int dy);
    cv::Rect clampRoiToFrame(const cv::Rect& roi, const cv::Size& frameSize) const;
    cv::Mat preprocessFrame(const cv::Mat& frame, const cv::Rect& roi);
    cv::Mat makePreviewFrame(const cv::Mat& originalFrame,
                             const cv::Mat& processedFrame,
                             const cv::Rect& roi) const;
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
    bool displayDecodedPacket(const AVPacket* packet);
    void logAvError(const std::string& prefix, int errnum) const;
    bool initHardwareDecode();
    static AVPixelFormat getHardwareDecodeFormat(AVCodecContext* ctx, const AVPixelFormat* pix_fmts);

private:
    CaptureConfig m_captureConfig;
    PreprocessConfig m_preprocessConfig;
    EncodeConfig m_encodeConfig;
    TransportConfig m_transportConfig;

    cv::VideoCapture m_cap;
    int m_udpSocket = -1;
    sockaddr_in m_serverAddr = {};
    int m_serialHandle = -1;

#ifdef HAVE_HIKROBOT_SDK
    std::unique_ptr<HikRobotCamera> m_hikrobotCamera;
#endif

    const AVCodec* m_codec = nullptr;
    AVCodecContext* m_codecContext = nullptr;
    AVFrame* m_yuvFrame = nullptr;
    AVFrame* m_encodeSwFrame = nullptr;
    AVFrame* m_encodeHwFrame = nullptr;
    AVPacket* m_packet = nullptr;
    SwsContext* m_swsContext = nullptr;
    const AVCodec* m_decodeCodec = nullptr;
    AVCodecContext* m_decodeContext = nullptr;
    AVFrame* m_decodeFrame = nullptr;
    SwsContext* m_decodeSwsContext = nullptr;
    AVBufferRef* m_decodeHwDeviceCtx = nullptr;
    AVBufferRef* m_encodeHwDeviceCtx = nullptr;
    AVBufferRef* m_encodeHwFramesCtx = nullptr;
    int m_decodeSrcWidth = 0;
    int m_decodeSrcHeight = 0;
    bool m_useHardwareEncoder = false;
    AVPixelFormat m_encoderInputPixelFormat = AV_PIX_FMT_YUV420P;

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
    std::atomic<uint64_t> m_totalEncodeMicros{0};
    uint64_t m_captureIndex = 0;

    cv::Rect m_runtimeRoi;
    bool m_runtimeRoiInitialized = false;
    cv::Mat m_previousGray;
    std::deque<cv::Mat> m_trailHistory;
};
