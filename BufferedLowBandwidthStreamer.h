#pragma once

#include "LowBandwidthStreamer.h"

#include <deque>

class BufferedLowBandwidthStreamer {
public:
    BufferedLowBandwidthStreamer(const CaptureConfig& captureConfig = {},
                                 const PreprocessConfig& preprocessConfig = {},
                                 const EncodeConfig& encodeConfig = {},
                                 const TransportConfig& transportConfig = {});
    ~BufferedLowBandwidthStreamer();

    bool initialize();
    void startStreaming();
    void stopStreaming();
    bool isStreaming() const { return m_isStreaming.load(); }
    uint64_t totalBytesSent() const { return m_totalBytesSent.load(); }
    uint64_t totalPacketsSent() const { return m_totalPacketsSent.load(); }
    uint64_t totalFramesSent() const { return m_totalFramesSent.load(); }
    uint64_t droppedFrames() const { return m_droppedFrames.load(); }

private:
    struct QueuedFrame {
        cv::Mat frame;
        uint64_t index = 0;
    };

    struct EncodedFrame {
        uint32_t frameIndex = 0;
        std::vector<uint8_t> data;
        size_t nextOffset = 0;
        uint16_t nextPacketIndex = 0;
    };

    bool initCamera();
    bool initSocket();
    bool initSerial();
    bool initEncoder();
    void cleanup();

    void captureLoop();
    void processLoop();
    void sendLoop();

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
    void enqueueEncodedFrame(const uint8_t* data, int size, uint32_t frameIndex);
    bool sendOnePacket(EncodedFrame& frame);
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
    std::thread m_sendThread;

    std::mutex m_frameQueueMutex;
    std::condition_variable m_frameQueueCv;
    std::deque<QueuedFrame> m_frameQueue;
    const size_t m_maxFrameQueueSize = 3;

    std::mutex m_sendQueueMutex;
    std::condition_variable m_sendQueueCv;
    std::deque<EncodedFrame> m_sendQueue;
    size_t m_backlogBytes = 0;
    size_t m_maxBacklogBytes = 0;

    std::atomic<bool> m_initialized{false};
    std::atomic<bool> m_isStreaming{false};
    std::atomic<bool> m_stopRequested{false};
    std::atomic<uint32_t> m_outputFrameCounter{0};
    std::atomic<uint64_t> m_totalBytesSent{0};
    std::atomic<uint64_t> m_totalPacketsSent{0};
    std::atomic<uint64_t> m_totalFramesSent{0};
    std::atomic<uint64_t> m_droppedFrames{0};
    uint64_t m_captureIndex = 0;

    cv::Mat m_previousGray;
    std::deque<cv::Mat> m_trailHistory;
};
