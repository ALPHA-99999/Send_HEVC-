#include "lb_streamer.h"
#include "lb_streamer_utils.h"

#ifdef HAVE_HIKROBOT_SDK
#include "hikrobot_camera.hpp"
#endif

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <sstream>

#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

LowBandwidthStreamer::LowBandwidthStreamer(const CaptureConfig& captureConfig,
                                           const PreprocessConfig& preprocessConfig,
                                           const EncodeConfig& encodeConfig,
                                           const TransportConfig& transportConfig)
    : m_captureConfig(captureConfig),
      m_preprocessConfig(preprocessConfig),
      m_encodeConfig(encodeConfig),
      m_transportConfig(transportConfig) {}

LowBandwidthStreamer::~LowBandwidthStreamer() {
    stopStreaming();
    cleanup();
}

bool LowBandwidthStreamer::initialize() {
    if (m_initialized.load()) {
        return true;
    }

    // 先把传输通道准备好，再继续初始化摄像头和编码器。
    bool transportReady = false;
#if LOW_BANDWIDTH_TRANSPORT_MODE == LOW_BANDWIDTH_TRANSPORT_SERIAL
    transportReady = initSerial();
#else
    transportReady = initSocket();
#endif

    if (!initCamera() || !transportReady || !initEncoder() || !initDecoder()) {
        cleanup();
        return false;
    }

    m_initialized = true;
    return true;
}

bool LowBandwidthStreamer::initCamera() {
    if (m_captureConfig.sourceType == CaptureSourceType::Hikrobot) {
#ifdef HAVE_HIKROBOT_SDK
        m_hikrobotCamera = std::make_unique<HikRobotCamera>(m_captureConfig.hikrobotExposureMs,
                                                             m_captureConfig.hikrobotGain,
                                                             m_captureConfig.hikrobotVidPid,
                                                             m_captureConfig.hikrobotRecordDir,
                                                             m_captureConfig.hikrobotRecordFps,
                                                             m_captureConfig.hikrobotRecordRawVideo);
        std::cout << "Hikrobot camera initialized";
        if (!m_captureConfig.hikrobotVidPid.empty()) {
            std::cout << " (" << m_captureConfig.hikrobotVidPid << ")";
        }
        std::cout << std::endl;
        return true;
#else
        std::cerr << "Hikrobot support is not enabled in this build. "
                  << "Reconfigure CMake with the Hikrobot SDK path to enable it." << std::endl;
        return false;
#endif
    }

    if (!m_captureConfig.videoPath.empty()) {
        m_cap.open(m_captureConfig.videoPath);
        if (!m_cap.isOpened()) {
            std::cerr << "Failed to open video file: " << m_captureConfig.videoPath << std::endl;
            return false;
        }

        // 文件源按指定帧率节奏播放，避免一口气读完整个 AVI。
        if (m_captureConfig.captureFps > 0) {
            m_cap.set(cv::CAP_PROP_FPS, m_captureConfig.captureFps);
        }

        const double actualWidth = m_cap.get(cv::CAP_PROP_FRAME_WIDTH);
        const double actualHeight = m_cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        const double actualFps = m_cap.get(cv::CAP_PROP_FPS);
        std::cout << "Video file initialized: " << m_captureConfig.videoPath << " "
                  << actualWidth << "x" << actualHeight
                  << " @" << actualFps << "fps" << std::endl;
        return true;
    }

    int openedIndex = m_captureConfig.cameraIndex;
    if (!lb_streamer_detail::tryOpenCameraDevice(m_cap, openedIndex)) {
        for (int idx = 0; idx <= 5; ++idx) {
            if (idx == m_captureConfig.cameraIndex) {
                continue;
            }
            if (lb_streamer_detail::tryOpenCameraDevice(m_cap, idx)) {
                openedIndex = idx;
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

    m_captureConfig.cameraIndex = openedIndex;
    lb_streamer_detail::applyCameraParams(m_cap,
                                          m_captureConfig.inputWidth,
                                          m_captureConfig.inputHeight,
                                          m_captureConfig.captureFps);

    const double actualWidth = m_cap.get(cv::CAP_PROP_FRAME_WIDTH);
    const double actualHeight = m_cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    const double actualFps = m_cap.get(cv::CAP_PROP_FPS);

    std::cout << "Camera initialized: cap(" << m_captureConfig.cameraIndex << ") "
              << actualWidth << "x" << actualHeight
              << " @" << actualFps << "fps" << std::endl;
    return true;
}

bool LowBandwidthStreamer::initSocket() {
    m_udpSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (m_udpSocket == kInvalidSocket) {
        std::cerr << "Socket creation failed: " << std::strerror(errno) << std::endl;
        return false;
    }

    int broadcast = 1;
    setsockopt(m_udpSocket, SOL_SOCKET, SO_BROADCAST,
               reinterpret_cast<const char*>(&broadcast), sizeof(broadcast));

    memset(&m_serverAddr, 0, sizeof(m_serverAddr));
    m_serverAddr.sin_family = AF_INET;
    m_serverAddr.sin_port = htons(static_cast<uint16_t>(m_transportConfig.udpPort));
    if (inet_pton(AF_INET, m_transportConfig.udpAddress.c_str(), &m_serverAddr.sin_addr) != 1) {
        std::cerr << "Invalid UDP address: " << m_transportConfig.udpAddress << std::endl;
        return false;
    }

    return true;
}

bool LowBandwidthStreamer::initSerial() {
    m_serialHandle = ::open(m_transportConfig.serialPort.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (m_serialHandle < 0) {
        std::cerr << "Failed to open serial port " << m_transportConfig.serialPort
                  << ", error: " << std::strerror(errno) << std::endl;
        return false;
    }

    if (!lb_streamer_detail::configureLinuxSerialPort(m_serialHandle, m_transportConfig.serialBaudRate)) {
        ::close(m_serialHandle);
        m_serialHandle = kInvalidSerialHandle;
        return false;
    }

    std::cout << "Serial initialized: " << m_transportConfig.serialPort
              << " @ " << m_transportConfig.serialBaudRate << " baud" << std::endl;
    return true;
}

bool LowBandwidthStreamer::initEncoder() {
    m_codec = avcodec_find_encoder_by_name(m_encodeConfig.codecName.c_str());
    if (!m_codec) {
        std::cerr << "Encoder not found: " << m_encodeConfig.codecName << std::endl;
        return false;
    }

    m_codecContext = avcodec_alloc_context3(m_codec);
    if (!m_codecContext) {
        std::cerr << "Could not allocate codec context" << std::endl;
        return false;
    }

    m_codecContext->bit_rate = static_cast<int64_t>(m_encodeConfig.bitrateKbps) * 1000;
    m_codecContext->rc_min_rate = m_codecContext->bit_rate;
    m_codecContext->rc_max_rate = m_codecContext->bit_rate;
    m_codecContext->rc_buffer_size = m_codecContext->bit_rate;
    m_codecContext->width = m_preprocessConfig.outputWidth;
    m_codecContext->height = m_preprocessConfig.outputHeight;
    m_codecContext->time_base = AVRational{1, std::max(1, m_encodeConfig.fps)};
    m_codecContext->framerate = AVRational{std::max(1, m_encodeConfig.fps), 1};
    m_codecContext->gop_size = std::max(1, m_encodeConfig.gopSize);
    m_codecContext->max_b_frames = std::max(0, m_encodeConfig.maxBFrames);
    m_codecContext->pix_fmt = lb_streamer_detail::chooseEncoderPixelFormat(m_encodeConfig.codecName);

    av_opt_set(m_codecContext->priv_data, "preset", m_encodeConfig.preset.c_str(), 0);
    if (!m_encodeConfig.tune.empty()) {
        av_opt_set(m_codecContext->priv_data, "tune", m_encodeConfig.tune.c_str(), 0);
    }
    av_opt_set(m_codecContext->priv_data, "repeat-headers", "1", 0);

    std::ostringstream x265Params;
    x265Params << "keyint=" << m_codecContext->gop_size
               << ":min-keyint=" << m_codecContext->gop_size
               << ":bframes=" << m_codecContext->max_b_frames
               << ":vbv-maxrate=" << m_encodeConfig.bitrateKbps
               << ":vbv-bufsize=" << m_encodeConfig.bitrateKbps
               << ":aq-mode=2:aq-strength=1.2";
    av_opt_set(m_codecContext->priv_data, "x265-params", x265Params.str().c_str(), 0);

    int ret = avcodec_open2(m_codecContext, m_codec, nullptr);
    if (ret < 0) {
        logAvError("Could not open codec", ret);
        return false;
    }

    m_yuvFrame = av_frame_alloc();
    m_packet = av_packet_alloc();
    if (!m_yuvFrame || !m_packet) {
        std::cerr << "Could not allocate frame or packet" << std::endl;
        return false;
    }

    m_yuvFrame->format = m_codecContext->pix_fmt;
    m_yuvFrame->width = m_codecContext->width;
    m_yuvFrame->height = m_codecContext->height;
    if (av_frame_get_buffer(m_yuvFrame, 32) < 0) {
        std::cerr << "Could not allocate frame buffer" << std::endl;
        return false;
    }

    m_swsContext = sws_getContext(m_preprocessConfig.outputWidth,
                                  m_preprocessConfig.outputHeight,
                                  AV_PIX_FMT_BGR24,
                                  m_codecContext->width,
                                  m_codecContext->height,
                                  m_codecContext->pix_fmt,
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

bool LowBandwidthStreamer::initDecoder() {
    m_decodeCodec = avcodec_find_decoder(m_codec->id);
    if (!m_decodeCodec) {
        std::cerr << "Decoder not found for codec id: " << m_codec->id << std::endl;
        return false;
    }

    m_decodeContext = avcodec_alloc_context3(m_decodeCodec);
    if (!m_decodeContext) {
        std::cerr << "Could not allocate decoder context" << std::endl;
        return false;
    }

    int ret = avcodec_open2(m_decodeContext, m_decodeCodec, nullptr);
    if (ret < 0) {
        logAvError("Could not open decoder", ret);
        return false;
    }

    m_decodeFrame = av_frame_alloc();
    if (!m_decodeFrame) {
        std::cerr << "Could not allocate decoder frame" << std::endl;
        return false;
    }

    return true;
}

void LowBandwidthStreamer::startStreaming() {
    if (!m_initialized.load() || m_isStreaming.load()) {
        return;
    }

    // 每次开始推流前清空队列和统计值，避免上一次状态残留。
    {
        std::lock_guard<std::mutex> lock(m_queueMutex);
        m_frameQueue.clear();
    }
    m_previousGray.release();
    m_trailHistory.clear();
    m_outputFrameCounter = 0;
    m_totalBytesSent = 0;
    m_totalPacketsSent = 0;
    m_totalFramesSent = 0;
    m_captureIndex = 0;
    m_runtimeRoi = cv::Rect();
    m_runtimeRoiInitialized = false;
    m_stopRequested = false;
    m_isStreaming = true;

    m_captureThread = std::thread(&LowBandwidthStreamer::captureLoop, this);
    m_processThread = std::thread(&LowBandwidthStreamer::processLoop, this);
}

void LowBandwidthStreamer::stopStreaming() {
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

void LowBandwidthStreamer::captureLoop() {
    int consecutiveFailures = 0;
    const auto fileFrameInterval = m_captureConfig.captureFps > 0
        ? std::chrono::milliseconds(std::max(1, 1000 / m_captureConfig.captureFps))
        : std::chrono::milliseconds(0);
    while (!m_stopRequested.load()) {
        const auto loopStart = std::chrono::steady_clock::now();
        cv::Mat frame;
        if (m_captureConfig.sourceType == CaptureSourceType::Hikrobot) {
#ifdef HAVE_HIKROBOT_SDK
            std::chrono::steady_clock::time_point timestamp;
            try {
                if (m_hikrobotCamera) {
                    m_hikrobotCamera->read(frame, timestamp);
                }
            } catch (const std::exception& e) {
                std::cerr << "Hikrobot read failed: " << e.what() << std::endl;
                m_stopRequested = true;
                m_queueCv.notify_all();
                break;
            }
#else
            std::cerr << "Hikrobot support is not available in this build." << std::endl;
            m_stopRequested = true;
            m_queueCv.notify_all();
            break;
#endif
        } else if (!m_cap.read(frame) || frame.empty()) {
            if (!m_captureConfig.videoPath.empty()) {
                std::cerr << "Video file reached end: " << m_captureConfig.videoPath << std::endl;
                m_stopRequested = true;
                m_queueCv.notify_all();
                break;
            }
            ++consecutiveFailures;
            if (consecutiveFailures == 1 || consecutiveFailures % 50 == 0) {
                std::cerr << "Failed to capture frame from cap("
                          << m_captureConfig.cameraIndex
                          << "), retry=" << consecutiveFailures << std::endl;
            }

            if (consecutiveFailures % 100 == 0) {
                m_cap.release();
                if (lb_streamer_detail::tryOpenCameraDevice(m_cap, m_captureConfig.cameraIndex)) {
                    lb_streamer_detail::applyCameraParams(m_cap,
                                                          m_captureConfig.inputWidth,
                                                          m_captureConfig.inputHeight,
                                                          m_captureConfig.captureFps);
                    consecutiveFailures = 0;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        consecutiveFailures = 0;

        std::lock_guard<std::mutex> lock(m_queueMutex);
        if (m_frameQueue.size() >= m_maxQueueSize) {
            m_frameQueue.pop_front();
        }
        m_frameQueue.push_back({frame, m_captureIndex++});
        m_queueCv.notify_one();

        if (!m_captureConfig.videoPath.empty() && fileFrameInterval.count() > 0) {
            const auto elapsed = std::chrono::steady_clock::now() - loopStart;
            const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
            if (elapsedMs < fileFrameInterval) {
                std::this_thread::sleep_for(fileFrameInterval - elapsedMs);
            }
        }
    }
}

void LowBandwidthStreamer::processLoop() {
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

        // 这里取的是当前运行时 ROI，方向键移动后会马上生效。
        const cv::Rect roi = currentRuntimeRoi(queued.frame.size());
        cv::Mat processed = preprocessFrame(queued.frame, roi);
        if (processed.empty()) {
            continue;
        }

        if (m_captureConfig.showPreview) {
            // 左边显示原图和 ROI 框，右边显示压缩后的处理结果。
            cv::Mat preview = makePreviewFrame(queued.frame, processed, roi);
            cv::imshow("LowBandwidth Preview", preview);
            const int key = cv::waitKeyEx(1);
            constexpr int kRoiStep = 20;
            // 只允许方向键移动 ROI，不改变 ROI 大小。
            auto isLeft = [](int k) {
                return k == 81 || k == 2424832 || k == 65361 || k == 0xFF51;
            };
            auto isUp = [](int k) {
                return k == 82 || k == 2490368 || k == 65362 || k == 0xFF52;
            };
            auto isRight = [](int k) {
                return k == 83 || k == 2555904 || k == 65363 || k == 0xFF53;
            };
            auto isDown = [](int k) {
                return k == 84 || k == 2621440 || k == 65364 || k == 0xFF54;
            };

            if (key == 27) {
                m_stopRequested = true;
                m_isStreaming = false;
                m_queueCv.notify_all();
                break;
            } else if (isLeft(key)) {
                moveRuntimeRoi(queued.frame.size(), -kRoiStep, 0);
            } else if (isUp(key)) {
                moveRuntimeRoi(queued.frame.size(), 0, -kRoiStep);
            } else if (isRight(key)) {
                moveRuntimeRoi(queued.frame.size(), kRoiStep, 0);
            } else if (isDown(key)) {
                moveRuntimeRoi(queued.frame.size(), 0, kRoiStep);
            }
        }

        const uint32_t frameIndex = m_outputFrameCounter.fetch_add(1);
        if (!encodeFrame(processed, frameIndex)) {
            std::cerr << "Failed to encode frame " << frameIndex << std::endl;
        }
    }

    flushEncoder();
    m_isStreaming = false;
}

void LowBandwidthStreamer::cleanup() {
    if (m_swsContext) {
        sws_freeContext(m_swsContext);
        m_swsContext = nullptr;
    }
    if (m_decodeSwsContext) {
        sws_freeContext(m_decodeSwsContext);
        m_decodeSwsContext = nullptr;
    }
    if (m_packet) {
        av_packet_free(&m_packet);
    }
    if (m_yuvFrame) {
        av_frame_free(&m_yuvFrame);
    }
    if (m_decodeFrame) {
        av_frame_free(&m_decodeFrame);
    }
    if (m_codecContext) {
        avcodec_free_context(&m_codecContext);
    }
    if (m_decodeContext) {
        avcodec_free_context(&m_decodeContext);
    }
    if (m_cap.isOpened()) {
        m_cap.release();
    }
#ifdef HAVE_HIKROBOT_SDK
    m_hikrobotCamera.reset();
    if (m_decodeSwsContext) {
        m_decodeSrcWidth = 0;
        m_decodeSrcHeight = 0;
    }
#endif
    if (m_udpSocket != kInvalidSocket) {
        ::close(m_udpSocket);
        m_udpSocket = kInvalidSocket;
    }
    if (m_serialHandle != kInvalidSerialHandle) {
        ::close(m_serialHandle);
        m_serialHandle = kInvalidSerialHandle;
    }
    m_initialized = false;
}
