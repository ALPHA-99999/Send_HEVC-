#pragma once

#include "lb_streaming_components.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

class WireDecoder;

class LowBandwidthStreamer {
public:
    LowBandwidthStreamer(const FrameSource::Config& captureConfig,
                         const FramePreprocessor::Config& preprocessConfig,
                         const FrameEncoder::Config& encodeConfig,
                         const TransportChannel::Config& transportConfig);
    ~LowBandwidthStreamer();

    bool initialize();
    void startStreaming();
    void stopStreaming();
    bool isStreaming() const { return m_isStreaming.load(); }
    uint64_t totalBytesSent() const;
    uint64_t totalPacketsSent() const;
    uint64_t totalFramesSent() const { return m_totalFramesSent.load(); }
    uint64_t totalEncodeMicros() const;

private:
    struct QueuedFrame {
        cv::Mat frame;
        uint64_t index = 0;
    };

    void captureLoop();
    void processLoop();
    void cleanup();
    bool handleCaptureFailure(const FrameSource::Config* captureConfig, int& consecutiveFailures);
    void enqueueCapturedFrame(cv::Mat frame);
    void throttleFilePlayback(const FrameSource::Config* captureConfig,
                              const std::chrono::steady_clock::time_point& loopStart,
                              std::chrono::milliseconds fileFrameInterval);

private:
    std::unique_ptr<TransportChannel> m_transport;
    std::unique_ptr<FrameSource> m_source;
    std::unique_ptr<FramePreprocessor> m_preprocessor;
    std::unique_ptr<FrameEncoder> m_encoder;
    std::unique_ptr<WireDecoder> m_decoder;

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
    std::atomic<uint64_t> m_totalFramesSent{0};
    uint64_t m_captureIndex = 0;
};
