#include "lb_streamer.h"

#include "lb_streaming_components.h"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

LowBandwidthStreamer::LowBandwidthStreamer(const FrameSource::Config& captureConfig,
                                           const FramePreprocessor::Config& preprocessConfig,
                                           const FrameEncoder::Config& encodeConfig,
                                           const TransportChannel::Config& transportConfig)
{
    m_transport = std::make_unique<TransportChannel>(transportConfig);
    m_source = std::make_unique<FrameSource>(captureConfig);
    m_preprocessor = std::make_unique<FramePreprocessor>(preprocessConfig);
    m_encoder = std::make_unique<FrameEncoder>(encodeConfig, preprocessConfig, transportConfig);
    m_decoder = std::make_unique<WireDecoder>();
}

LowBandwidthStreamer::~LowBandwidthStreamer()
{
    stopStreaming();
    cleanup();
}

uint64_t LowBandwidthStreamer::totalBytesSent() const
{
    return m_transport ? m_transport->totalBytesSent() : 0;
}

uint64_t LowBandwidthStreamer::totalPacketsSent() const
{
    return m_transport ? m_transport->totalPacketsSent() : 0;
}

uint64_t LowBandwidthStreamer::totalEncodeMicros() const
{
    return m_encoder ? m_encoder->totalEncodeMicros() : 0;
}

bool LowBandwidthStreamer::initialize()
{
    if (m_initialized.load()) {
        return true;
    }

    if (!m_transport->open() || !m_source->open() || !m_encoder->open() || !m_decoder->open()) {
        cleanup();
        return false;
    }

    m_encoder->setSendPacketCallback([this](const uint8_t* packet, size_t packetSize) {
        return m_transport->send(packet, packetSize);
    });

    m_initialized = true;
    return true;
}

void LowBandwidthStreamer::startStreaming()
{
    if (!m_initialized.load() || m_isStreaming.load()) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(m_queueMutex);
        m_frameQueue.clear();
    }

    if (m_preprocessor) {
        m_preprocessor->reset();
    }
    if (m_encoder) {
        m_encoder->resetStats();
    }

    m_outputFrameCounter = 0;
    m_captureIndex = 0;
    m_stopRequested = false;
    m_isStreaming = true;

    m_captureThread = std::thread(&LowBandwidthStreamer::captureLoop, this);
    m_processThread = std::thread(&LowBandwidthStreamer::processLoop, this);
}

void LowBandwidthStreamer::stopStreaming()
{
    if (!m_isStreaming.exchange(false) && !m_stopRequested.exchange(true)) {
        return;
    }

    m_stopRequested = true;
    m_queueCv.notify_all();

    if (m_captureThread.joinable()) {
        m_captureThread.join();
    }
    if (m_processThread.joinable()) {
        m_processThread.join();
    }

    cv::destroyAllWindows();
}

void LowBandwidthStreamer::captureLoop()
{
    int consecutiveFailures = 0;
    std::uint64_t rawFrameCount = 0;
    auto rawFpsWindowStart = std::chrono::steady_clock::now();
    const FrameSource::Config* sourceConfig = m_source ? &m_source->config() : nullptr;
    const auto fileFrameInterval = sourceConfig && sourceConfig->captureFps > 0
        ? std::chrono::milliseconds(std::max(1, 1000 / sourceConfig->captureFps))
        : std::chrono::milliseconds(0);

    while (!m_stopRequested.load()) {
        const auto loopStart = std::chrono::steady_clock::now();
        cv::Mat frame;
        std::chrono::steady_clock::time_point timestamp;
        if (!m_source || !m_source->read(frame, timestamp)) {
            if (handleCaptureFailure(sourceConfig, consecutiveFailures)) {
                break;
            }
            continue;
        }

        consecutiveFailures = 0;
        ++rawFrameCount;

        const auto now = std::chrono::steady_clock::now();
        const auto rawElapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - rawFpsWindowStart);

        // if (rawElapsedMs.count() >= 1000) {
        //     const double rawFps = static_cast<double>(rawFrameCount) * 1000.0 /
        //                           static_cast<double>(std::max<int64_t>(1, rawElapsedMs.count()));
        //     std::ostringstream oss;
        //     oss << std::fixed << std::setprecision(2) << rawFps;
        //     std::cerr << "[info] raw input fps: " << oss.str() << std::endl;
        //     rawFrameCount = 0;
        //     rawFpsWindowStart = now;
        // }

        enqueueCapturedFrame(std::move(frame));
        throttleFilePlayback(sourceConfig, loopStart, fileFrameInterval);
    }
}

bool LowBandwidthStreamer::handleCaptureFailure(const FrameSource::Config* captureConfig, int& consecutiveFailures)
{
    if (captureConfig && captureConfig->sourceType == CaptureSourceType::File) {
        if (!captureConfig->videoPath.empty()) {
            std::cerr << "Video file reached end: " << captureConfig->videoPath << std::endl;
        }
        m_stopRequested = true;
        m_queueCv.notify_all();
        return true;
    }

    ++consecutiveFailures;
    if (consecutiveFailures == 1 || consecutiveFailures % 50 == 0) {
        std::cerr << "Failed to capture frame from source, retry=" << consecutiveFailures << std::endl;
    }

    if (m_source && captureConfig && captureConfig->sourceType == CaptureSourceType::LocalCamera && consecutiveFailures % 100 == 0) {
        if (m_source->reopenCamera(m_source->cameraIndex())) {
            consecutiveFailures = 0;
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return false;
}

void LowBandwidthStreamer::enqueueCapturedFrame(cv::Mat frame)
{
    std::lock_guard<std::mutex> lock(m_queueMutex);
    if (m_frameQueue.size() >= m_maxQueueSize) {
        m_frameQueue.pop_front();
    }
    m_frameQueue.push_back({std::move(frame), m_captureIndex++});
    m_queueCv.notify_one();
}

void LowBandwidthStreamer::throttleFilePlayback(const FrameSource::Config* captureConfig,
                                                const std::chrono::steady_clock::time_point& loopStart,
                                                std::chrono::milliseconds fileFrameInterval)
{
    if (!captureConfig || captureConfig->videoPath.empty() || fileFrameInterval.count() <= 0) {
        return;
    }

    const auto elapsed = std::chrono::steady_clock::now() - loopStart;
    const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
    if (elapsedMs < fileFrameInterval) {
        std::this_thread::sleep_for(fileFrameInterval - elapsedMs);
    }
}

void LowBandwidthStreamer::processLoop()
{
    while (true) {
        QueuedFrame queued;
        {
            std::unique_lock<std::mutex> lock(m_queueMutex);
            m_queueCv.wait(lock, [this] {
                return m_stopRequested.load() || !m_frameQueue.empty();
            });

            if (m_frameQueue.empty()) {
                break;
            }

            queued = std::move(m_frameQueue.back());
            m_frameQueue.clear();
        }

        const cv::Rect roi = m_preprocessor->currentRuntimeRoi(queued.frame.size());
        cv::Mat processed = m_preprocessor->process(queued.frame, roi);
        if (processed.empty()) {
            continue;
        }

        if (m_source && m_source->config().showPreview) {
            cv::Mat preview = m_preprocessor->makePreviewFrame(queued.frame, processed, roi);
            cv::imshow("LowBandwidth Preview", preview);
            const int key = cv::waitKeyEx(1);
            constexpr int kRoiStep = 20;
            bool roiChanged = false;
            auto isEscape = [](int k) { return k == 27 || (k & 0xFF) == 27; };
            auto isLeft = [](int k) { return k == 81 || k == 2424832 || k == 65361 || k == 0xFF51 || (k & 0xFF) == 'a' || (k & 0xFF) == 'A'; };
            auto isUp = [](int k) { return k == 82 || k == 2490368 || k == 65362 || k == 0xFF52 || (k & 0xFF) == 'w' || (k & 0xFF) == 'W'; };
            auto isRight = [](int k) { return k == 83 || k == 2555904 || k == 65363 || k == 0xFF53 || (k & 0xFF) == 'd' || (k & 0xFF) == 'D'; };
            auto isDown = [](int k) { return k == 84 || k == 2621440 || k == 65364 || k == 0xFF54 || (k & 0xFF) == 's' || (k & 0xFF) == 'S'; };

            if (isEscape(key)) {
                m_stopRequested = true;
                m_isStreaming = false;
                m_queueCv.notify_all();
                break;
            } else if (isLeft(key)) {
                m_preprocessor->moveRuntimeRoi(queued.frame.size(), -kRoiStep, 0);
                roiChanged = true;
            } else if (isUp(key)) {
                m_preprocessor->moveRuntimeRoi(queued.frame.size(), 0, -kRoiStep);
                roiChanged = true;
            } else if (isRight(key)) {
                m_preprocessor->moveRuntimeRoi(queued.frame.size(), kRoiStep, 0);
                roiChanged = true;
            } else if (isDown(key)) {
                m_preprocessor->moveRuntimeRoi(queued.frame.size(), 0, kRoiStep);
                roiChanged = true;
            }

            if (roiChanged) {
                const cv::Rect updatedRoi = m_preprocessor->currentRuntimeRoi(queued.frame.size());
                processed = m_preprocessor->process(queued.frame, updatedRoi);
                cv::Mat updatedPreview = m_preprocessor->makePreviewFrame(queued.frame, processed, updatedRoi);
                cv::imshow("LowBandwidth Preview", updatedPreview);
            }
        }

        const uint32_t frameIndex = m_outputFrameCounter.fetch_add(1);
        if (!m_encoder->encodeFrame(processed, frameIndex)) {
            std::cerr << "Failed to encode frame " << frameIndex << std::endl;
        } else {
            m_totalFramesSent.fetch_add(1);
        }
    }

    if (!m_encoder->flush()) {
        std::cerr << "Failed to flush encoder" << std::endl;
    }
    m_isStreaming = false;
}

void LowBandwidthStreamer::cleanup()
{
    if (m_encoder) {
        m_encoder->close();
    }
    if (m_decoder) {
        m_decoder->close();
    }
    if (m_source) {
        m_source->close();
    }
    if (m_transport) {
        m_transport->close();
    }

    m_encoder.reset();
    m_decoder.reset();
    m_preprocessor.reset();
    m_source.reset();
    m_transport.reset();
    m_initialized = false;
}
