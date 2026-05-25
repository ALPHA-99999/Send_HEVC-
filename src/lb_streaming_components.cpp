#include "lb_streaming_components.h"

#include "hikrobot_camera.hpp"
#include "lb_streamer_utils.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <thread>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>

TransportChannel::TransportChannel(const Config& config)
    : m_config(config) {}

TransportChannel::~TransportChannel()
{
    close();
}

bool TransportChannel::openSocket()
{
    m_udpSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (m_udpSocket == kInvalidSocket) {
        std::cerr << "Socket creation failed: " << std::strerror(errno) << std::endl;
        return false;
    }

    int broadcast = 1;
    setsockopt(m_udpSocket, SOL_SOCKET, SO_BROADCAST,
               reinterpret_cast<const char*>(&broadcast), sizeof(broadcast));

    std::memset(&m_serverAddr, 0, sizeof(m_serverAddr));
    m_serverAddr.sin_family = AF_INET;
    m_serverAddr.sin_port = htons(static_cast<uint16_t>(m_config.udpPort));
    if (inet_pton(AF_INET, m_config.udpAddress.c_str(), &m_serverAddr.sin_addr) != 1) {
        std::cerr << "Invalid UDP address: " << m_config.udpAddress << std::endl;
        return false;
    }
    return true;
}

bool TransportChannel::openSerial()
{
    m_serialHandle = ::open(m_config.serialPort.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (m_serialHandle < 0) {
        std::cerr << "Failed to open serial port " << m_config.serialPort
                  << ", error: " << std::strerror(errno) << std::endl;
        return false;
    }

    if (!lb_streamer_detail::configureLinuxSerialPort(m_serialHandle, m_config.serialBaudRate)) {
        ::close(m_serialHandle);
        m_serialHandle = kInvalidSerialHandle;
        return false;
    }

    std::cout << "Serial initialized: " << m_config.serialPort
              << " @ " << m_config.serialBaudRate << " baud" << std::endl;
    return true;
}

bool TransportChannel::open()
{
    switch (m_config.mode) {
    case Mode::Udp:
        return openSocket();
    case Mode::Serial:
        return openSerial();
    }
    std::cerr << "Unknown transport mode" << std::endl;
    return false;
}

void TransportChannel::close()
{
    if (m_udpSocket != kInvalidSocket) {
        ::close(m_udpSocket);
        m_udpSocket = kInvalidSocket;
    }
    if (m_serialHandle != kInvalidSerialHandle) {
        ::close(m_serialHandle);
        m_serialHandle = kInvalidSerialHandle;
    }
}

bool TransportChannel::send(const uint8_t* data, size_t size)
{
    switch (m_config.mode) {
    case Mode::Serial:
        if (!lb_streamer_detail::writeSerialAll(m_serialHandle, data, size)) {
            return false;
        }
        if (m_config.serialInterPacketDelayMs > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(m_config.serialInterPacketDelayMs));
        }
        break;
    case Mode::Udp: {
        const int sent = sendto(m_udpSocket,
                                reinterpret_cast<const char*>(data),
                                static_cast<int>(size),
                                0,
                                reinterpret_cast<sockaddr*>(&m_serverAddr),
                                sizeof(m_serverAddr));
        if (sent < 0) {
            std::cerr << "sendto failed: " << std::strerror(errno) << std::endl;
            return false;
        }
        break;
    }
    default:
        std::cerr << "Unknown transport mode" << std::endl;
        return false;
    }
    m_totalBytesSent.fetch_add(static_cast<uint64_t>(size));
    m_totalPacketsSent.fetch_add(1);
    return true;
}

FrameSource::FrameSource(const Config& config)
    : m_config(config) {}

FrameSource::~FrameSource()
{
    close();
}

bool FrameSource::open()
{
    m_opened = false;
    if (m_config.sourceType == CaptureSourceType::Hikrobot) {
        m_hikrobotCamera = std::make_unique<HikRobotCamera>(m_config.hikrobotExposureMs,
                                                            m_config.hikrobotGain,
                                                            m_config.hikrobotVidPid,
                                                            m_config.hikrobotRecordDir,
                                                            m_config.hikrobotRecordFps,
                                                            m_config.hikrobotRecordRawVideo);
        std::cout << "Hikrobot camera initialized";
        if (!m_config.hikrobotVidPid.empty()) {
            std::cout << " (" << m_config.hikrobotVidPid << ")";
        }
        std::cout << std::endl;
        m_opened = true;
        return true;
    }

    if (!m_config.videoPath.empty()) {
        m_cap.open(m_config.videoPath);
        if (!m_cap.isOpened()) {
            std::cerr << "Failed to open video file: " << m_config.videoPath << std::endl;
            return false;
        }
        if (m_config.captureFps > 0) {
            m_cap.set(cv::CAP_PROP_FPS, m_config.captureFps);
        }
        const double actualWidth = m_cap.get(cv::CAP_PROP_FRAME_WIDTH);
        const double actualHeight = m_cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        const double actualFps = m_cap.get(cv::CAP_PROP_FPS);
        std::cout << "Video file initialized: " << m_config.videoPath << " "
                  << actualWidth << "x" << actualHeight
                  << " @" << actualFps << "fps" << std::endl;
        m_opened = true;
        return true;
    }

    m_cameraIndex = m_config.cameraIndex;
    if (!lb_streamer_detail::tryOpenCameraDevice(m_cap, m_cameraIndex)) {
        for (int idx = 0; idx <= 5; ++idx) {
            if (idx == m_cameraIndex) {
                continue;
            }
            if (lb_streamer_detail::tryOpenCameraDevice(m_cap, idx)) {
                m_cameraIndex = idx;
                break;
            }
        }
    }

    if (!m_cap.isOpened()) {
        std::cerr << "Failed to open camera. Tried indexes 0-5. "
                  << "On Ubuntu, confirm /dev/video* exists and your user is in group 'video'."
                  << std::endl;
        return false;
    }

    lb_streamer_detail::applyCameraParams(m_cap,
                                          m_config.inputWidth,
                                          m_config.inputHeight,
                                          m_config.captureFps);

    const double actualWidth = m_cap.get(cv::CAP_PROP_FRAME_WIDTH);
    const double actualHeight = m_cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    const double actualFps = m_cap.get(cv::CAP_PROP_FPS);
    std::cout << "Camera initialized: cap(" << m_cameraIndex << ") "
              << actualWidth << "x" << actualHeight
              << " @" << actualFps << "fps" << std::endl;
    m_opened = true;
    return true;
}

void FrameSource::close()
{
    if (m_cap.isOpened()) {
        m_cap.release();
    }
    m_hikrobotCamera.reset();
    m_opened = false;
}

bool FrameSource::reopenCamera(int index)
{
    if (m_config.sourceType != CaptureSourceType::LocalCamera) {
        return false;
    }

    m_cap.release();
    if (!lb_streamer_detail::tryOpenCameraDevice(m_cap, index)) {
        return false;
    }
    lb_streamer_detail::applyCameraParams(m_cap,
                                          m_config.inputWidth,
                                          m_config.inputHeight,
                                          m_config.captureFps);
    m_cameraIndex = index;
    return true;
}

bool FrameSource::read(cv::Mat& frame, std::chrono::steady_clock::time_point& timestamp)
{
    timestamp = std::chrono::steady_clock::now();
    if (m_config.sourceType == CaptureSourceType::Hikrobot) {
        if (!m_hikrobotCamera) {
            return false;
        }
        m_hikrobotCamera->read(frame, timestamp);
        return !frame.empty();
    }

    if (!m_cap.read(frame) || frame.empty()) {
        return false;
    }
    return true;
}

FramePreprocessor::FramePreprocessor(const Config& config)
    : m_config(config) {}

void FramePreprocessor::reset()
{
    m_runtimeRoi = cv::Rect();
    m_runtimeRoiInitialized = false;
    m_previousGray.release();
    m_trailHistory.clear();
}

cv::Rect FramePreprocessor::computeEffectiveRoi(const cv::Size& frameSize) const
{
    if (!m_config.enableRoi) {
        return cv::Rect(0, 0, frameSize.width, frameSize.height);
    }

    cv::Rect roi = m_config.roi;
    if (roi.width <= 0 || roi.height <= 0) {
        const int side = std::min(frameSize.width, frameSize.height);
        roi = cv::Rect((frameSize.width - side) / 2, (frameSize.height - side) / 2, side, side);
    }

    roi &= cv::Rect(0, 0, frameSize.width, frameSize.height);
    if (roi.width <= 0 || roi.height <= 0) {
        return cv::Rect(0, 0, frameSize.width, frameSize.height);
    }
    return roi;
}

cv::Rect FramePreprocessor::currentRuntimeRoi(const cv::Size& frameSize)
{
    if (!m_runtimeRoiInitialized || m_runtimeRoi.width <= 0 || m_runtimeRoi.height <= 0) {
        m_runtimeRoi = computeEffectiveRoi(frameSize);
        m_runtimeRoiInitialized = true;
        return m_runtimeRoi;
    }

    m_runtimeRoi = clampRoiToFrame(m_runtimeRoi, frameSize);
    m_runtimeRoiInitialized = true;
    return m_runtimeRoi;
}

void FramePreprocessor::moveRuntimeRoi(const cv::Size& frameSize, int dx, int dy)
{
    cv::Rect roi = currentRuntimeRoi(frameSize);
    roi.x += dx;
    roi.y += dy;
    m_runtimeRoi = clampRoiToFrame(roi, frameSize);
    m_runtimeRoiInitialized = true;
}

cv::Rect FramePreprocessor::clampRoiToFrame(const cv::Rect& roi, const cv::Size& frameSize) const
{
    cv::Rect bounded = roi;
    if (bounded.width <= 0 || bounded.height <= 0) {
        return computeEffectiveRoi(frameSize);
    }

    bounded.width = std::min(bounded.width, frameSize.width);
    bounded.height = std::min(bounded.height, frameSize.height);
    bounded.x = std::max(0, std::min(bounded.x, frameSize.width - bounded.width));
    bounded.y = std::max(0, std::min(bounded.y, frameSize.height - bounded.height));
    bounded &= cv::Rect(0, 0, frameSize.width, frameSize.height);
    if (bounded.width <= 0 || bounded.height <= 0) {
        return computeEffectiveRoi(frameSize);
    }
    return bounded;
}

cv::Mat FramePreprocessor::buildMotionMask(const cv::Mat& grayFrame, double& motionRatio)
{
    cv::Mat motionMask(grayFrame.size(), CV_8UC1, cv::Scalar(0));
    if (m_previousGray.empty()) {
        m_previousGray = grayFrame.clone();
        motionRatio = 0.0;
        return motionMask;
    }

    cv::Mat diff;
    cv::absdiff(grayFrame, m_previousGray, diff);
    cv::threshold(diff, motionMask, m_config.motionThreshold, 255, cv::THRESH_BINARY);

    if (m_config.erodeSize > 0) {
        const cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_RECT,
            cv::Size(m_config.erodeSize * 2 + 1, m_config.erodeSize * 2 + 1));
        cv::erode(motionMask, motionMask, kernel);
    }

    if (m_config.dilateSize > 0) {
        const cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_RECT,
            cv::Size(m_config.dilateSize * 2 + 1, m_config.dilateSize * 2 + 1));
        cv::dilate(motionMask, motionMask, kernel);
    }

    motionRatio = static_cast<double>(cv::countNonZero(motionMask)) /
                  static_cast<double>(motionMask.rows * motionMask.cols);
    m_previousGray = grayFrame.clone();
    return motionMask;
}

void FramePreprocessor::applyStaticSuppression(const cv::Mat& source,
                                               const cv::Mat& grayFrame,
                                               const cv::Mat& motionMask,
                                               cv::Mat& output) const
{
    cv::Mat grayBgr;
    cv::cvtColor(grayFrame, grayBgr, cv::COLOR_GRAY2BGR);

    int blurKernel = std::max(1, m_config.blurKernel);
    if (blurKernel % 2 == 0) {
        ++blurKernel;
    }
    cv::GaussianBlur(grayBgr, output, cv::Size(blurKernel, blurKernel), 0.0);
    source.copyTo(output, motionMask);
}

void FramePreprocessor::applyCenterProtection(const cv::Mat& source, cv::Mat& output) const
{
    if (m_config.centerProtectSize <= 0) {
        return;
    }

    const int protectWidth = std::min(m_config.centerProtectSize, source.cols);
    const int protectHeight = std::min(m_config.centerProtectSize, source.rows);
    const cv::Rect centerRect((source.cols - protectWidth) / 2,
                              (source.rows - protectHeight) / 2,
                              protectWidth,
                              protectHeight);
    source(centerRect).copyTo(output(centerRect));
}

void FramePreprocessor::applyTrail(const cv::Mat& grayFrame,
                                   const cv::Mat& motionMask,
                                   double motionRatio,
                                   cv::Mat& output)
{
    if (m_config.trailLength <= 0) {
        return;
    }

    if (motionRatio > m_config.trailDisableRatio) {
        m_trailHistory.clear();
        return;
    }

    cv::Mat trailCandidate(grayFrame.size(), CV_8UC1, cv::Scalar(0));
    grayFrame.copyTo(trailCandidate, motionMask);

    if (cv::countNonZero(trailCandidate) == 0) {
        return;
    }

    m_trailHistory.push_back(trailCandidate);
    while (static_cast<int>(m_trailHistory.size()) > m_config.trailLength) {
        m_trailHistory.pop_front();
    }

    cv::Mat trail = m_trailHistory.front().clone();
    for (size_t i = 1; i < m_trailHistory.size(); ++i) {
        cv::max(trail, m_trailHistory[i], trail);
    }

    cv::Mat trailBgr;
    cv::cvtColor(trail, trailBgr, cv::COLOR_GRAY2BGR);
    cv::max(output, trailBgr, output);
}

cv::Mat FramePreprocessor::process(const cv::Mat& frame, const cv::Rect& roi)
{
    cv::Mat roiFrame = frame(roi).clone();
    cv::Mat resized;
    cv::resize(roiFrame, resized,
               cv::Size(m_config.outputWidth, m_config.outputHeight),
               0.0, 0.0, cv::INTER_AREA);

    cv::Mat grayFrame;
    cv::cvtColor(resized, grayFrame, cv::COLOR_BGR2GRAY);

    double motionRatio = 0.0;
    cv::Mat motionMask = buildMotionMask(grayFrame, motionRatio);

    cv::Mat output;
    applyStaticSuppression(resized, grayFrame, motionMask, output);
    applyCenterProtection(resized, output);
    applyTrail(grayFrame, motionMask, motionRatio, output);
    applyCenterProtection(resized, output);
    return output;
}

cv::Mat FramePreprocessor::makePreviewFrame(const cv::Mat& originalFrame,
                                            const cv::Mat& processedFrame,
                                            const cv::Rect& roi) const
{
    if (originalFrame.empty() || processedFrame.empty()) {
        return processedFrame;
    }

    const cv::Size originalPreviewTarget(960, 540);
    double scale = 1.0;
    cv::Point offset;
    cv::Mat originalPreview = lb_streamer_detail::fitIntoCanvas(originalFrame, originalPreviewTarget, scale, offset);

    const double roiScaleX = scale;
    const double roiScaleY = scale;
    cv::Rect previewRoi(
        static_cast<int>(std::lround(roi.x * roiScaleX)) + offset.x,
        static_cast<int>(std::lround(roi.y * roiScaleY)) + offset.y,
        static_cast<int>(roi.width * roiScaleX),
        static_cast<int>(roi.height * roiScaleY));
    previewRoi &= cv::Rect(0, 0, originalPreview.cols, originalPreview.rows);
    if (previewRoi.width > 0 && previewRoi.height > 0) {
        cv::rectangle(originalPreview, previewRoi, cv::Scalar(0, 255, 255), 2);
    }
    cv::putText(originalPreview, "Original",
                cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
    cv::putText(originalPreview, "Arrows move ROI, ESC quit",
                cv::Point(12, 58), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    cv::putText(originalPreview,
                "ROI x=" + std::to_string(roi.x) + " y=" + std::to_string(roi.y) +
                    " w=" + std::to_string(roi.width) + " h=" + std::to_string(roi.height),
                cv::Point(12, 86), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(0, 255, 255), 1, cv::LINE_AA);

    double processedScale = 1.0;
    cv::Point processedOffset;
    cv::Mat processedPreview = lb_streamer_detail::fitIntoCanvas(processedFrame, originalPreview.size(), processedScale, processedOffset);
    cv::putText(processedPreview, "Processed",
                cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                cv::Scalar(0, 255, 255), 2, cv::LINE_AA);

    const int canvasWidth = originalPreview.cols + processedPreview.cols;
    const int canvasHeight = std::max(originalPreview.rows, processedPreview.rows);
    cv::Mat canvas(canvasHeight, canvasWidth, originalPreview.type(), cv::Scalar::all(0));
    originalPreview.copyTo(canvas(cv::Rect(0, 0, originalPreview.cols, originalPreview.rows)));
    processedPreview.copyTo(canvas(cv::Rect(originalPreview.cols, 0,
                                            processedPreview.cols, processedPreview.rows)));

    constexpr int kMaxPreviewWidth = 1600;
    constexpr int kMaxPreviewHeight = 900;
    if (canvas.cols > kMaxPreviewWidth || canvas.rows > kMaxPreviewHeight) {
        const double shrink = std::min(static_cast<double>(kMaxPreviewWidth) / static_cast<double>(canvas.cols),
                                       static_cast<double>(kMaxPreviewHeight) / static_cast<double>(canvas.rows));
        cv::Mat scaled;
        cv::resize(canvas, scaled,
                   cv::Size(std::max(1, static_cast<int>(std::lround(canvas.cols * shrink))),
                            std::max(1, static_cast<int>(std::lround(canvas.rows * shrink)))),
                   0.0, 0.0, cv::INTER_AREA);
        return scaled;
    }

    return canvas;
}

WirePacketizer::WirePacketizer(const TransportChannel::Config& config)
    : m_config(config) {}

void WirePacketizer::reset()
{
    m_encodedChunks.clear();
    m_encodedChunkOffset = 0;
    m_nextPacketSeq = 0;
}

bool WirePacketizer::sendWirePacket(const uint8_t* payload,
                                    size_t payloadLen,
                                    uint8_t flags,
                                    uint16_t syncOffset,
                                    const std::function<bool(const uint8_t* packet, size_t packetSize)>& sendPacket)
{
    const std::size_t wirePacketSize = static_cast<std::size_t>(std::max(1, m_config.wirePacketSize));
    if (wirePacketSize < lb_streamer_detail::kStreamHeaderSize) {
        return false;
    }
    std::array<uint8_t, lb_streamer_detail::kWirePacketSize> packet{};
    if (wirePacketSize != packet.size()) {
        std::cerr << "wirePacketSize must be " << packet.size() << " bytes" << std::endl;
        return false;
    }

    packet[0] = static_cast<uint8_t>((lb_streamer_detail::kStreamMagic >> 24) & 0xFF);
    packet[1] = static_cast<uint8_t>((lb_streamer_detail::kStreamMagic >> 16) & 0xFF);
    packet[2] = static_cast<uint8_t>((lb_streamer_detail::kStreamMagic >> 8) & 0xFF);
    packet[3] = static_cast<uint8_t>(lb_streamer_detail::kStreamMagic & 0xFF);
    packet[4] = lb_streamer_detail::kStreamVersion;
    packet[5] = flags;
    packet[6] = static_cast<uint8_t>((lb_streamer_detail::kStreamHeaderSize >> 8) & 0xFF);
    packet[7] = static_cast<uint8_t>(lb_streamer_detail::kStreamHeaderSize & 0xFF);
    packet[8] = static_cast<uint8_t>((m_nextPacketSeq >> 24) & 0xFF);
    packet[9] = static_cast<uint8_t>((m_nextPacketSeq >> 16) & 0xFF);
    packet[10] = static_cast<uint8_t>((m_nextPacketSeq >> 8) & 0xFF);
    packet[11] = static_cast<uint8_t>(m_nextPacketSeq & 0xFF);
    packet[12] = static_cast<uint8_t>((payloadLen >> 8) & 0xFF);
    packet[13] = static_cast<uint8_t>(payloadLen & 0xFF);
    packet[14] = static_cast<uint8_t>((syncOffset >> 8) & 0xFF);
    packet[15] = static_cast<uint8_t>(syncOffset & 0xFF);

    if (payloadLen > 0 && payload != nullptr) {
        std::memcpy(packet.data() + lb_streamer_detail::kStreamHeaderSize, payload, payloadLen);
    }

    if (payloadLen < lb_streamer_detail::kWirePacketSize - lb_streamer_detail::kStreamHeaderSize) {
        std::memset(packet.data() + lb_streamer_detail::kStreamHeaderSize + payloadLen,
                    0,
                    lb_streamer_detail::kWirePacketSize - lb_streamer_detail::kStreamHeaderSize - payloadLen);
    }

    using clock = std::chrono::steady_clock;
    static clock::time_point s_nextAllowedPacketSendTime = clock::now();
    const bool throttlePackets = m_config.maxSendHz > 0;
    const int maxPacketHz = std::max(1, m_config.maxSendHz);
    const auto minPacketInterval = std::chrono::microseconds(1000000 / maxPacketHz);
    if (throttlePackets) {
        const auto now = clock::now();
        if (now < s_nextAllowedPacketSendTime) {
            std::this_thread::sleep_until(s_nextAllowedPacketSendTime);
        }
        s_nextAllowedPacketSendTime = clock::now() + minPacketInterval;
    }

    if (!sendPacket(packet.data(), packet.size())) {
        return false;
    }
    ++m_nextPacketSeq;
    return true;
}

bool WirePacketizer::pushEncodedPayload(const uint8_t* data,
                                        int size,
                                        bool keyframe,
                                        const std::function<bool(const uint8_t* packet, size_t packetSize)>& sendPacket)
{
    if (!data || size <= 0) {
        return true;
    }

    EncodedChunk chunk;
    chunk.data.assign(data, data + size);
    chunk.keyframe = keyframe;
    m_encodedChunks.push_back(std::move(chunk));
    return flush(false, sendPacket);
}

bool WirePacketizer::flush(bool finalFlush,
                           const std::function<bool(const uint8_t* packet, size_t packetSize)>& sendPacket)
{
    const std::size_t wirePacketSize = static_cast<std::size_t>(std::max(1, m_config.wirePacketSize));
    if (wirePacketSize < lb_streamer_detail::kStreamHeaderSize) {
        std::cerr << "wirePacketSize is smaller than the stream header" << std::endl;
        return false;
    }

    const std::size_t payloadCapacity = wirePacketSize - lb_streamer_detail::kStreamHeaderSize;
    while (true) {
        std::size_t bufferedBytes = 0;
        bool firstChunk = true;
        for (const auto& chunk : m_encodedChunks) {
            if (firstChunk) {
                const std::size_t consumed = std::min(m_encodedChunkOffset, chunk.data.size());
                bufferedBytes += chunk.data.size() - consumed;
                firstChunk = false;
            } else {
                bufferedBytes += chunk.data.size();
            }
        }
        if (!finalFlush && bufferedBytes < payloadCapacity) {
            break;
        }
        if (bufferedBytes == 0 && !finalFlush) {
            break;
        }

        const std::size_t bytesToCopy = finalFlush
            ? std::min(payloadCapacity, bufferedBytes)
            : payloadCapacity;
        std::vector<uint8_t> payload(bytesToCopy);
        std::size_t copied = 0;
        bool syncPacket = false;
        uint16_t syncOffset = 0xFFFF;

        while (copied < bytesToCopy && !m_encodedChunks.empty()) {
            EncodedChunk& chunk = m_encodedChunks.front();
            if (m_encodedChunkOffset >= chunk.data.size()) {
                m_encodedChunks.pop_front();
                m_encodedChunkOffset = 0;
                continue;
            }

            const std::size_t chunkOffset = m_encodedChunkOffset;
            const std::size_t chunkRemaining = chunk.data.size() - chunkOffset;
            const std::size_t toCopy = std::min(bytesToCopy - copied, chunkRemaining);
            if (chunk.keyframe && chunkOffset == 0 && !syncPacket) {
                syncPacket = true;
                syncOffset = static_cast<uint16_t>(copied);
            }

            std::memcpy(payload.data() + copied,
                        chunk.data.data() + chunkOffset,
                        toCopy);
            copied += toCopy;
            m_encodedChunkOffset += toCopy;

            if (m_encodedChunkOffset >= chunk.data.size()) {
                m_encodedChunks.pop_front();
                m_encodedChunkOffset = 0;
            }
        }

        if (copied == 0) {
            if (!finalFlush) {
                break;
            }
        }

        uint8_t flags = 0;
        if (syncPacket) {
            flags |= lb_streamer_detail::kStreamFlagKeyframe;
        }
        if (finalFlush && m_encodedChunks.empty()) {
            flags |= lb_streamer_detail::kStreamFlagEndOfStream;
        }

        if (!sendWirePacket(payload.data(), copied, flags, syncOffset, sendPacket)) {
            return false;
        }

        if (!finalFlush && copied < payloadCapacity) {
            break;
        }
        if (finalFlush && m_encodedChunks.empty()) {
            break;
        }
    }

    if (finalFlush && m_encodedChunks.empty()) {
        if (!sendWirePacket(nullptr, 0, lb_streamer_detail::kStreamFlagEndOfStream, 0xFFFF, sendPacket)) {
            return false;
        }
    }
    return true;
}

FrameEncoder::FrameEncoder(const Config& config,
                           const FramePreprocessor::Config& preprocessConfig,
                           const TransportChannel::Config& transportConfig)
    : m_config(config),
      m_preprocessConfig(preprocessConfig),
      m_transportConfig(transportConfig),
      m_packetizer(transportConfig) {}

FrameEncoder::~FrameEncoder()
{
    close();
}

void FrameEncoder::setSendPacketCallback(std::function<bool(const uint8_t* packet, size_t packetSize)> sendPacket)
{
    m_sendPacket = std::move(sendPacket);
}

void FrameEncoder::logAvError(const std::string& prefix, int errnum) const
{
    char buffer[AV_ERROR_MAX_STRING_SIZE] = {};
    av_strerror(errnum, buffer, sizeof(buffer));
    std::cerr << prefix << ": " << buffer << std::endl;
}

bool FrameEncoder::initHardwareEncoder()
{
    AVBufferRef* deviceCtx = nullptr;
    const int ret = av_hwdevice_ctx_create(&deviceCtx,
                                           AV_HWDEVICE_TYPE_VAAPI,
                                           "/dev/dri/renderD128",
                                           nullptr,
                                           0);
    if (ret < 0) {
        logAvError("VAAPI hardware encoder device init failed", ret);
        return false;
    }

    m_encodeHwDeviceCtx = deviceCtx;
    m_encodeHwFramesCtx = av_hwframe_ctx_alloc(m_encodeHwDeviceCtx);
    if (!m_encodeHwFramesCtx) {
        std::cerr << "Could not allocate VAAPI hw frame context" << std::endl;
        return false;
    }

    auto* framesCtx = reinterpret_cast<AVHWFramesContext*>(m_encodeHwFramesCtx->data);
    framesCtx->format = AV_PIX_FMT_VAAPI;
    framesCtx->sw_format = AV_PIX_FMT_NV12;
    framesCtx->width = m_codecContext->width;
    framesCtx->height = m_codecContext->height;
    framesCtx->initial_pool_size = 20;

    const int initRet = av_hwframe_ctx_init(m_encodeHwFramesCtx);
    if (initRet < 0) {
        logAvError("Could not initialize VAAPI hw frame context", initRet);
        return false;
    }

    m_codecContext->hw_device_ctx = av_buffer_ref(m_encodeHwDeviceCtx);
    m_codecContext->hw_frames_ctx = av_buffer_ref(m_encodeHwFramesCtx);
    m_codecContext->pix_fmt = AV_PIX_FMT_VAAPI;
    m_codecContext->sw_pix_fmt = AV_PIX_FMT_NV12;
    return true;
}

bool FrameEncoder::open()
{
    const std::string codecName = lb_streamer_detail::normalizeEncoderName(m_config.codecName);
    m_useHardwareEncoder = lb_streamer_detail::isHardwareVaapiEncoder(codecName);
    m_encoderInputPixelFormat = m_useHardwareEncoder ? AV_PIX_FMT_NV12 : AV_PIX_FMT_YUV420P;

    m_codec = avcodec_find_encoder_by_name(codecName.c_str());
    if (!m_codec) {
        std::cerr << "Encoder not found: " << codecName << std::endl;
        return false;
    }

    m_codecContext = avcodec_alloc_context3(m_codec);
    if (!m_codecContext) {
        std::cerr << "Could not allocate codec context" << std::endl;
        return false;
    }

    m_codecContext->bit_rate = static_cast<int64_t>(m_config.bitrateKbps) * 1000;
    m_codecContext->rc_min_rate = m_codecContext->bit_rate;
    m_codecContext->rc_max_rate = m_codecContext->bit_rate;
    m_codecContext->rc_buffer_size = m_codecContext->bit_rate;
    m_codecContext->width = m_preprocessConfig.outputWidth;
    m_codecContext->height = m_preprocessConfig.outputHeight;
    m_codecContext->time_base = AVRational{1, std::max(1, m_config.fps)};
    m_codecContext->framerate = AVRational{std::max(1, m_config.fps), 1};
    m_codecContext->gop_size = std::max(1, m_config.gopSize);
    m_codecContext->max_b_frames = m_useHardwareEncoder ? 0 : std::max(0, m_config.maxBFrames);
    m_codecContext->pix_fmt = m_useHardwareEncoder ? AV_PIX_FMT_VAAPI
                                                   : lb_streamer_detail::chooseEncoderPixelFormat(codecName);

    if (m_useHardwareEncoder) {
        if (!initHardwareEncoder()) {
            std::cerr << "Could not initialize hardware encoder" << std::endl;
            return false;
        }
    } else {
        av_opt_set(m_codecContext->priv_data, "preset", m_config.preset.c_str(), 0);
        if (!m_config.tune.empty()) {
            av_opt_set(m_codecContext->priv_data, "tune", m_config.tune.c_str(), 0);
        }
        av_opt_set(m_codecContext->priv_data, "repeat-headers", "1", 0);
        std::ostringstream x265Params;
        x265Params << "keyint=" << m_codecContext->gop_size
                   << ":min-keyint=" << m_codecContext->gop_size
                   << ":bframes=" << m_codecContext->max_b_frames
                   << ":vbv-maxrate=" << m_config.bitrateKbps
                   << ":vbv-bufsize=" << m_config.bitrateKbps
                   << ":aq-mode=2:aq-strength=1.2";
        av_opt_set(m_codecContext->priv_data, "x265-params", x265Params.str().c_str(), 0);
    }

    const int ret = avcodec_open2(m_codecContext, m_codec, nullptr);
    if (ret < 0) {
        logAvError("Could not open codec", ret);
        return false;
    }

    m_yuvFrame = av_frame_alloc();
    m_encodeSwFrame = av_frame_alloc();
    m_encodeHwFrame = av_frame_alloc();
    m_packet = av_packet_alloc();
    if (!m_yuvFrame || !m_encodeSwFrame || !m_encodeHwFrame || !m_packet) {
        std::cerr << "Could not allocate frame or packet" << std::endl;
        return false;
    }

    if (m_useHardwareEncoder) {
        m_encodeSwFrame->format = m_encoderInputPixelFormat;
        m_encodeSwFrame->width = m_codecContext->width;
        m_encodeSwFrame->height = m_codecContext->height;
        if (av_frame_get_buffer(m_encodeSwFrame, 32) < 0) {
            std::cerr << "Could not allocate software encoder frame buffer" << std::endl;
            return false;
        }

        m_encodeHwFrame->format = AV_PIX_FMT_VAAPI;
        m_encodeHwFrame->width = m_codecContext->width;
        m_encodeHwFrame->height = m_codecContext->height;
    } else {
        m_yuvFrame->format = m_codecContext->pix_fmt;
        m_yuvFrame->width = m_codecContext->width;
        m_yuvFrame->height = m_codecContext->height;
        if (av_frame_get_buffer(m_yuvFrame, 32) < 0) {
            std::cerr << "Could not allocate frame buffer" << std::endl;
            return false;
        }
    }

    m_swsContext = sws_getContext(m_preprocessConfig.outputWidth,
                                  m_preprocessConfig.outputHeight,
                                  AV_PIX_FMT_BGR24,
                                  m_codecContext->width,
                                  m_codecContext->height,
                                  m_encoderInputPixelFormat,
                                  SWS_BILINEAR,
                                  nullptr,
                                  nullptr,
                                  nullptr);
    if (!m_swsContext) {
        std::cerr << "Could not initialize sws context" << std::endl;
        return false;
    }

    return true;
}

void FrameEncoder::close()
{
    if (m_swsContext) {
        sws_freeContext(m_swsContext);
        m_swsContext = nullptr;
    }
    if (m_packet) {
        av_packet_free(&m_packet);
    }
    if (m_yuvFrame) {
        av_frame_free(&m_yuvFrame);
    }
    if (m_encodeSwFrame) {
        av_frame_free(&m_encodeSwFrame);
    }
    if (m_encodeHwFrame) {
        av_frame_free(&m_encodeHwFrame);
    }
    if (m_encodeHwFramesCtx) {
        av_buffer_unref(&m_encodeHwFramesCtx);
    }
    if (m_encodeHwDeviceCtx) {
        av_buffer_unref(&m_encodeHwDeviceCtx);
    }
    if (m_codecContext) {
        avcodec_free_context(&m_codecContext);
    }
}

void FrameEncoder::resetStats()
{
    m_totalEncodeMicros = 0;
    m_packetizer.reset();
}

bool FrameEncoder::encodeFrame(const cv::Mat& frame, uint32_t frameIndex)
{
    if (!m_codecContext || !m_packet || !m_sendPacket) {
        return false;
    }

    AVFrame* submitFrame = nullptr;
    if (m_useHardwareEncoder) {
        if (!m_encodeSwFrame || !m_encodeHwFrame || !m_encodeHwFramesCtx) {
            return false;
        }

        int ret = av_frame_make_writable(m_encodeSwFrame);
        if (ret < 0) {
            logAvError("Frame not writable", ret);
            return false;
        }

        uint8_t* srcSlice[] = {const_cast<uint8_t*>(frame.data)};
        int srcStride[] = {static_cast<int>(frame.step)};
        sws_scale(m_swsContext,
                  srcSlice,
                  srcStride,
                  0,
                  frame.rows,
                  m_encodeSwFrame->data,
                  m_encodeSwFrame->linesize);

        av_frame_unref(m_encodeHwFrame);
        ret = av_hwframe_get_buffer(m_encodeHwFramesCtx, m_encodeHwFrame, 0);
        if (ret < 0) {
            logAvError("Could not get VAAPI frame buffer", ret);
            return false;
        }

        ret = av_hwframe_transfer_data(m_encodeHwFrame, m_encodeSwFrame, 0);
        if (ret < 0) {
            logAvError("Could not transfer frame to VAAPI", ret);
            return false;
        }

        m_encodeHwFrame->pts = frameIndex;
        submitFrame = m_encodeHwFrame;
    } else {
        if (!m_yuvFrame) {
            return false;
        }

        int ret = av_frame_make_writable(m_yuvFrame);
        if (ret < 0) {
            logAvError("Frame not writable", ret);
            return false;
        }

        uint8_t* srcSlice[] = {const_cast<uint8_t*>(frame.data)};
        int srcStride[] = {static_cast<int>(frame.step)};
        sws_scale(m_swsContext,
                  srcSlice,
                  srcStride,
                  0,
                  frame.rows,
                  m_yuvFrame->data,
                  m_yuvFrame->linesize);

        m_yuvFrame->pts = frameIndex;
        submitFrame = m_yuvFrame;
    }

    const auto encodeStart = std::chrono::steady_clock::now();
    int ret = avcodec_send_frame(m_codecContext, submitFrame);
    if (ret < 0) {
        logAvError("Error sending frame to encoder", ret);
        return false;
    }

    while (ret >= 0) {
        ret = avcodec_receive_packet(m_codecContext, m_packet);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            break;
        }
        if (ret < 0) {
            logAvError("Error during encoding", ret);
            return false;
        }

        const bool keyframe = (m_packet->flags & AV_PKT_FLAG_KEY) != 0;
        if (!m_packetizer.pushEncodedPayload(m_packet->data, m_packet->size, keyframe, m_sendPacket)) {
            av_packet_unref(m_packet);
            return false;
        }
        av_packet_unref(m_packet);
    }

    const auto encodeEnd = std::chrono::steady_clock::now();
    const auto encodeMicros = std::chrono::duration_cast<std::chrono::microseconds>(encodeEnd - encodeStart).count();
    if (encodeMicros > 0) {
        m_totalEncodeMicros.fetch_add(static_cast<uint64_t>(encodeMicros));
    }

    return true;
}

bool FrameEncoder::flush()
{
    if (!m_codecContext || !m_packet) {
        return true;
    }

    int ret = avcodec_send_frame(m_codecContext, nullptr);
    if (ret < 0 && ret != AVERROR_EOF) {
        logAvError("Error flushing encoder", ret);
        return false;
    }

    while (true) {
        ret = avcodec_receive_packet(m_codecContext, m_packet);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            break;
        }
        if (ret < 0) {
            logAvError("Error receiving flushed packet", ret);
            return false;
        }

        const bool keyframe = (m_packet->flags & AV_PKT_FLAG_KEY) != 0;
        if (!m_packetizer.pushEncodedPayload(m_packet->data, m_packet->size, keyframe, m_sendPacket)) {
            av_packet_unref(m_packet);
            return false;
        }
        av_packet_unref(m_packet);
    }

    return m_packetizer.flush(true, m_sendPacket);
}

bool WireDecoder::open()
{
    return true;
}

void WireDecoder::close()
{
}
