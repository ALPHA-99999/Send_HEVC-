#include "LowBandwidthStreamer.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <sstream>

#ifndef _WIN32
#include <fcntl.h>
#include <termios.h>
#endif

namespace {
constexpr int kHeaderSize = 8;
constexpr uint8_t kSerialFrameSof = 0xA5;
constexpr uint8_t kSerialCmdIdLow = 0x10;
constexpr uint8_t kSerialCmdIdHigh = 0x03;
constexpr int kFixedSerialPayloadSize = 300;
constexpr int kSerialHeaderSize = 5;
constexpr int kSerialCmdSize = 2;
constexpr int kSerialCrc16Size = 2;

uint8_t getCrc8CheckSum(const uint8_t* message, uint32_t length, uint8_t crc)
{
    constexpr uint8_t polynomial = 0x07;
    if (!message) {
        return crc;
    }

    for (uint32_t i = 0; i < length; ++i) {
        crc ^= message[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ polynomial)
                               : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

uint16_t getCrc16CheckSum(const uint8_t* message, uint32_t length, uint16_t crc)
{
    constexpr uint16_t polynomial = 0x1021;
    if (!message) {
        return crc;
    }

    for (uint32_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(message[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ polynomial)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

bool writeSerialAll(serial_handle_t serialHandle, const uint8_t* data, size_t size)
{
#ifdef _WIN32
    if (serialHandle == kInvalidSerialHandle) {
        return false;
    }

    size_t totalWritten = 0;
    while (totalWritten < size) {
        DWORD written = 0;
        if (!WriteFile(serialHandle,
                       data + totalWritten,
                       static_cast<DWORD>(size - totalWritten),
                       &written,
                       nullptr)) {
            std::cerr << "WriteFile failed: " << GetLastError() << std::endl;
            return false;
        }
        if (written == 0) {
            std::cerr << "WriteFile wrote zero bytes" << std::endl;
            return false;
        }
        totalWritten += written;
    }

    return true;
#else
    if (serialHandle == kInvalidSerialHandle) {
        return false;
    }

    size_t totalWritten = 0;
    while (totalWritten < size) {
        const ssize_t written = ::write(serialHandle,
                                        data + totalWritten,
                                        size - totalWritten);
        if (written < 0) {
            if (errno == EINTR) {
                continue;
            }
            std::cerr << "write(serial) failed: " << std::strerror(errno) << std::endl;
            return false;
        }
        if (written == 0) {
            std::cerr << "write(serial) wrote zero bytes" << std::endl;
            return false;
        }
        totalWritten += static_cast<size_t>(written);
    }
    return true;
#endif
}

#ifndef _WIN32
speed_t baudToTermios(std::uint32_t baudRate)
{
    switch (baudRate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return B115200;
    }
}

bool configureLinuxSerialPort(int fd, std::uint32_t baudRate)
{
    termios tty = {};
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "tcgetattr failed: " << std::strerror(errno) << std::endl;
        return false;
    }

    cfmakeraw(&tty);
    const speed_t speed = baudToTermios(baudRate);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr failed: " << std::strerror(errno) << std::endl;
        return false;
    }

    tcflush(fd, TCIOFLUSH);
    return true;
}
#endif

AVPixelFormat chooseEncoderPixelFormat(const std::string& codecName)
{
    if (codecName.find("_qsv") != std::string::npos) {
        return AV_PIX_FMT_NV12;
    }
    return AV_PIX_FMT_YUV420P;
}

bool tryOpenCameraDevice(cv::VideoCapture& cap, int cameraIndex)
{
#ifdef __linux__
    const int backends[] = {cv::CAP_V4L2, cv::CAP_GSTREAMER, cv::CAP_ANY};
#else
    const int backends[] = {cv::CAP_ANY};
#endif
    for (int backend : backends) {
        if (cap.open(cameraIndex, backend)) {
            return true;
        }
    }
    return false;
}

void applyCameraParams(cv::VideoCapture& cap, int width, int height, int fps)
{
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap.set(cv::CAP_PROP_FPS, fps);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
}
}

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

#if LOW_BANDWIDTH_TRANSPORT_MODE == LOW_BANDWIDTH_TRANSPORT_UDP
    if (!socket_init()) {
        std::cerr << "Socket runtime initialization failed" << std::endl;
        return false;
    }
#endif

    bool transportReady = false;
#if LOW_BANDWIDTH_TRANSPORT_MODE == LOW_BANDWIDTH_TRANSPORT_SERIAL || \
    LOW_BANDWIDTH_TRANSPORT_MODE == LOW_BANDWIDTH_TRANSPORT_SERIAL_FIXED300
    transportReady = initSerial();
#else
    transportReady = initSocket();
#endif

    if (!initCamera() || !transportReady || !initEncoder()) {
        cleanup();
        return false;
    }

    m_initialized = true;
    return true;
}

bool LowBandwidthStreamer::initCamera() {
    int openedIndex = m_captureConfig.cameraIndex;
    if (!tryOpenCameraDevice(m_cap, openedIndex)) {
        for (int idx = 0; idx <= 5; ++idx) {
            if (idx == m_captureConfig.cameraIndex) {
                continue;
            }
            if (tryOpenCameraDevice(m_cap, idx)) {
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
    applyCameraParams(m_cap,
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
        std::cerr << "Socket creation failed: " << socket_last_error() << std::endl;
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
#ifdef _WIN32
    std::string serialPath = m_transportConfig.serialPort;
    if (serialPath.rfind("\\\\.\\", 0) != 0) {
        serialPath = "\\\\.\\" + serialPath;
    }

    m_serialHandle = CreateFileA(serialPath.c_str(),
                                 GENERIC_READ | GENERIC_WRITE,
                                 0,
                                 nullptr,
                                 OPEN_EXISTING,
                                 0,
                                 nullptr);
    if (m_serialHandle == kInvalidSerialHandle) {
        std::cerr << "Failed to open serial port " << m_transportConfig.serialPort
                  << ", error: " << GetLastError() << std::endl;
        return false;
    }

    DCB dcb = {};
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(m_serialHandle, &dcb)) {
        std::cerr << "GetCommState failed, error: " << GetLastError() << std::endl;
        return false;
    }

    dcb.BaudRate = m_transportConfig.serialBaudRate;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fTXContinueOnXoff = TRUE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fAbortOnError = FALSE;

    if (!SetCommState(m_serialHandle, &dcb)) {
        std::cerr << "SetCommState failed, error: " << GetLastError() << std::endl;
        return false;
    }

    COMMTIMEOUTS timeouts = {};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 100;
    if (!SetCommTimeouts(m_serialHandle, &timeouts)) {
        std::cerr << "SetCommTimeouts failed, error: " << GetLastError() << std::endl;
        return false;
    }

    SetupComm(m_serialHandle, 4096, 4096);
    PurgeComm(m_serialHandle, PURGE_RXCLEAR | PURGE_TXCLEAR);

    std::cout << "Serial initialized: " << m_transportConfig.serialPort
              << " @ " << m_transportConfig.serialBaudRate << " baud" << std::endl;
    return true;
#else
    m_serialHandle = ::open(m_transportConfig.serialPort.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (m_serialHandle < 0) {
        std::cerr << "Failed to open serial port " << m_transportConfig.serialPort
                  << ", error: " << std::strerror(errno) << std::endl;
        return false;
    }

    if (!configureLinuxSerialPort(m_serialHandle, m_transportConfig.serialBaudRate)) {
        ::close(m_serialHandle);
        m_serialHandle = kInvalidSerialHandle;
        return false;
    }

    std::cout << "Serial initialized: " << m_transportConfig.serialPort
              << " @ " << m_transportConfig.serialBaudRate << " baud" << std::endl;
    return true;
#endif
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
    m_codecContext->pix_fmt = chooseEncoderPixelFormat(m_encodeConfig.codecName);

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

void LowBandwidthStreamer::startStreaming() {
    if (!m_initialized.load() || m_isStreaming.load()) {
        return;
    }

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
    while (!m_stopRequested.load()) {
        cv::Mat frame;
        if (!m_cap.read(frame) || frame.empty()) {
            ++consecutiveFailures;
            if (consecutiveFailures == 1 || consecutiveFailures % 50 == 0) {
                std::cerr << "Failed to capture frame from cap("
                          << m_captureConfig.cameraIndex
                          << "), retry=" << consecutiveFailures << std::endl;
            }

            if (consecutiveFailures % 100 == 0) {
                m_cap.release();
                if (tryOpenCameraDevice(m_cap, m_captureConfig.cameraIndex)) {
                    applyCameraParams(m_cap,
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

        cv::Mat processed = preprocessFrame(queued.frame);
        if (processed.empty()) {
            continue;
        }

        if (m_captureConfig.showPreview) {
            cv::imshow("LowBandwidth Preview", processed);
            if (cv::waitKey(1) == 27) {
                m_stopRequested = true;
                m_isStreaming = false;
                m_queueCv.notify_all();
                break;
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

cv::Rect LowBandwidthStreamer::computeEffectiveRoi(const cv::Size& frameSize) const {
    if (!m_preprocessConfig.enableRoi) {
        return cv::Rect(0, 0, frameSize.width, frameSize.height);
    }

    cv::Rect roi = m_preprocessConfig.roi;
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

cv::Mat LowBandwidthStreamer::preprocessFrame(const cv::Mat& frame) {
    const cv::Rect roi = computeEffectiveRoi(frame.size());
    cv::Mat roiFrame = frame(roi).clone();

    cv::Mat resized;
    cv::resize(roiFrame, resized,
               cv::Size(m_preprocessConfig.outputWidth, m_preprocessConfig.outputHeight),
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

cv::Mat LowBandwidthStreamer::buildMotionMask(const cv::Mat& grayFrame, double& motionRatio) {
    cv::Mat motionMask(grayFrame.size(), CV_8UC1, cv::Scalar(0));
    if (m_previousGray.empty()) {
        m_previousGray = grayFrame.clone();
        motionRatio = 0.0;
        return motionMask;
    }

    cv::Mat diff;
    cv::absdiff(grayFrame, m_previousGray, diff);
    cv::threshold(diff, motionMask, m_preprocessConfig.motionThreshold, 255, cv::THRESH_BINARY);

    if (m_preprocessConfig.erodeSize > 0) {
        const cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_RECT,
            cv::Size(m_preprocessConfig.erodeSize * 2 + 1, m_preprocessConfig.erodeSize * 2 + 1));
        cv::erode(motionMask, motionMask, kernel);
    }

    if (m_preprocessConfig.dilateSize > 0) {
        const cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_RECT,
            cv::Size(m_preprocessConfig.dilateSize * 2 + 1, m_preprocessConfig.dilateSize * 2 + 1));
        cv::dilate(motionMask, motionMask, kernel);
    }

    motionRatio = static_cast<double>(cv::countNonZero(motionMask)) /
                  static_cast<double>(motionMask.rows * motionMask.cols);
    m_previousGray = grayFrame.clone();
    return motionMask;
}

void LowBandwidthStreamer::applyStaticSuppression(const cv::Mat& source,
                                                  const cv::Mat& grayFrame,
                                                  const cv::Mat& motionMask,
                                                  cv::Mat& output) const {
    cv::Mat grayBgr;
    cv::cvtColor(grayFrame, grayBgr, cv::COLOR_GRAY2BGR);

    int blurKernel = std::max(1, m_preprocessConfig.blurKernel);
    if (blurKernel % 2 == 0) {
        ++blurKernel;
    }
    cv::GaussianBlur(grayBgr, output, cv::Size(blurKernel, blurKernel), 0.0);

    source.copyTo(output, motionMask);
}

void LowBandwidthStreamer::applyCenterProtection(const cv::Mat& source, cv::Mat& output) const {
    if (m_preprocessConfig.centerProtectSize <= 0) {
        return;
    }

    const int protectWidth = std::min(m_preprocessConfig.centerProtectSize, source.cols);
    const int protectHeight = std::min(m_preprocessConfig.centerProtectSize, source.rows);
    const cv::Rect centerRect((source.cols - protectWidth) / 2,
                              (source.rows - protectHeight) / 2,
                              protectWidth,
                              protectHeight);
    source(centerRect).copyTo(output(centerRect));
}

void LowBandwidthStreamer::applyTrail(const cv::Mat& grayFrame,
                                      const cv::Mat& motionMask,
                                      double motionRatio,
                                      cv::Mat& output) {
    if (m_preprocessConfig.trailLength <= 0) {
        return;
    }

    if (motionRatio > m_preprocessConfig.trailDisableRatio) {
        m_trailHistory.clear();
        return;
    }

    cv::Mat trailCandidate(grayFrame.size(), CV_8UC1, cv::Scalar(0));
    grayFrame.copyTo(trailCandidate, motionMask);

    if (cv::countNonZero(trailCandidate) == 0) {
        return;
    }

    m_trailHistory.push_back(trailCandidate);
    while (static_cast<int>(m_trailHistory.size()) > m_preprocessConfig.trailLength) {
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

bool LowBandwidthStreamer::encodeFrame(const cv::Mat& frame, uint32_t frameIndex) {
    if (!m_codecContext || !m_yuvFrame || !m_packet) {
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

    ret = avcodec_send_frame(m_codecContext, m_yuvFrame);
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

        if (!sendEncodedPayload(m_packet->data, m_packet->size, frameIndex)) {
            av_packet_unref(m_packet);
            return false;
        }
        av_packet_unref(m_packet);
    }

    return true;
}

bool LowBandwidthStreamer::flushEncoder() {
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

        const uint32_t frameIndex = m_outputFrameCounter.load();
        if (!sendEncodedPayload(m_packet->data, m_packet->size, frameIndex)) {
            av_packet_unref(m_packet);
            return false;
        }
        av_packet_unref(m_packet);
    }

    return true;
}

bool LowBandwidthStreamer::sendEncodedPayload(const uint8_t* data, int size, uint32_t frameIndex) {
    if (!data || size <= 0) {
        return true;
    }

#if LOW_BANDWIDTH_TRANSPORT_MODE == LOW_BANDWIDTH_TRANSPORT_SERIAL || \
    LOW_BANDWIDTH_TRANSPORT_MODE == LOW_BANDWIDTH_TRANSPORT_SERIAL_FIXED300
    using clock = std::chrono::steady_clock;
    static clock::time_point s_nextAllowedPacketSendTime = clock::now();
    const int maxPacketHz = std::max(1, m_transportConfig.maxSendHz);
    const auto minPacketInterval = std::chrono::microseconds(1000000 / maxPacketHz);

    auto waitForPacketSlot = [&]() {
        const auto now = clock::now();
        if (now < s_nextAllowedPacketSendTime) {
            std::this_thread::sleep_until(s_nextAllowedPacketSendTime);
        }
        s_nextAllowedPacketSendTime = clock::now() + minPacketInterval;
    };
#endif

#if LOW_BANDWIDTH_TRANSPORT_MODE == LOW_BANDWIDTH_TRANSPORT_SERIAL_FIXED300
    constexpr int kInnerPacketSize = kFixedSerialPayloadSize;
    constexpr int kInnerPayloadSize = kInnerPacketSize - kHeaderSize;
    const int chunkCount = (size + kInnerPayloadSize - 1) / kInnerPayloadSize;
    uint64_t bytesSentForFrame = 0;
    static uint8_t serialSeq = 0;

    for (int packetIndex = 0; packetIndex < chunkCount; ++packetIndex) {
        const int offset = packetIndex * kInnerPayloadSize;
        const int chunkSize = std::min(kInnerPayloadSize, size - offset);

        std::vector<uint8_t> innerPacket(kInnerPacketSize, 0);
        innerPacket[0] = static_cast<uint8_t>((frameIndex >> 8) & 0xFF);
        innerPacket[1] = static_cast<uint8_t>(frameIndex & 0xFF);
        innerPacket[2] = static_cast<uint8_t>((packetIndex >> 8) & 0xFF);
        innerPacket[3] = static_cast<uint8_t>(packetIndex & 0xFF);
        innerPacket[4] = static_cast<uint8_t>((size >> 24) & 0xFF);
        innerPacket[5] = static_cast<uint8_t>((size >> 16) & 0xFF);
        innerPacket[6] = static_cast<uint8_t>((size >> 8) & 0xFF);
        innerPacket[7] = static_cast<uint8_t>(size & 0xFF);
        memcpy(innerPacket.data() + kHeaderSize, data + offset, chunkSize);

        const uint16_t dataLength = static_cast<uint16_t>(innerPacket.size());
        const uint16_t frameLength = static_cast<uint16_t>(
            kSerialHeaderSize + kSerialCmdSize + dataLength + kSerialCrc16Size);
        std::vector<uint8_t> serialFrame(frameLength, 0);

        serialFrame[0] = kSerialFrameSof;
        serialFrame[1] = static_cast<uint8_t>(dataLength & 0xFF);
        serialFrame[2] = static_cast<uint8_t>((dataLength >> 8) & 0xFF);
        serialFrame[3] = serialSeq++;
        serialFrame[4] = getCrc8CheckSum(serialFrame.data(), 4, 0xFF);
        serialFrame[5] = kSerialCmdIdLow;
        serialFrame[6] = kSerialCmdIdHigh;
        memcpy(serialFrame.data() + 7, innerPacket.data(), innerPacket.size());

        const uint16_t crc16 = getCrc16CheckSum(serialFrame.data(), frameLength - 2, 0xFFFF);
        serialFrame[frameLength - 2] = static_cast<uint8_t>(crc16 & 0xFF);
        serialFrame[frameLength - 1] = static_cast<uint8_t>((crc16 >> 8) & 0xFF);

        waitForPacketSlot();
        if (!writeSerialAll(m_serialHandle, serialFrame.data(), serialFrame.size())) {
            return false;
        }
        if (m_transportConfig.serialInterPacketDelayMs > 0 && packetIndex + 1 < chunkCount) {
            sleep_ms(m_transportConfig.serialInterPacketDelayMs);
        }

        bytesSentForFrame += static_cast<uint64_t>(serialFrame.size());
    }

    m_totalBytesSent.fetch_add(bytesSentForFrame);
    m_totalPacketsSent.fetch_add(static_cast<uint64_t>(chunkCount));
    m_totalFramesSent.fetch_add(1);
    return true;
#else
    const int payloadSize = std::max(1, m_transportConfig.payloadSize);
    const int chunkCount = (size + payloadSize - 1) / payloadSize;
    uint64_t bytesSentForFrame = 0;

    for (int packetIndex = 0; packetIndex < chunkCount; ++packetIndex) {
        const int offset = packetIndex * payloadSize;
        const int chunkSize = std::min(payloadSize, size - offset);

        std::vector<uint8_t> packet(kHeaderSize + chunkSize);
        packet[0] = static_cast<uint8_t>((frameIndex >> 8) & 0xFF);
        packet[1] = static_cast<uint8_t>(frameIndex & 0xFF);
        packet[2] = static_cast<uint8_t>((packetIndex >> 8) & 0xFF);
        packet[3] = static_cast<uint8_t>(packetIndex & 0xFF);
        packet[4] = static_cast<uint8_t>((size >> 24) & 0xFF);
        packet[5] = static_cast<uint8_t>((size >> 16) & 0xFF);
        packet[6] = static_cast<uint8_t>((size >> 8) & 0xFF);
        packet[7] = static_cast<uint8_t>(size & 0xFF);
        memcpy(packet.data() + kHeaderSize, data + offset, chunkSize);

        bool sentOk = false;
#if LOW_BANDWIDTH_TRANSPORT_MODE == LOW_BANDWIDTH_TRANSPORT_SERIAL
        waitForPacketSlot();
        if (!writeSerialAll(m_serialHandle, packet.data(), packet.size())) {
            return false;
        }
        if (m_transportConfig.serialInterPacketDelayMs > 0 && packetIndex + 1 < chunkCount) {
            sleep_ms(m_transportConfig.serialInterPacketDelayMs);
        }
        sentOk = true;
#else
     //   waitForPacketSlot();
        const int sent = sendto(m_udpSocket,
                                reinterpret_cast<const char*>(packet.data()),
                                static_cast<int>(packet.size()),
                                0,
                                reinterpret_cast<sockaddr*>(&m_serverAddr),
                                sizeof(m_serverAddr));
        if (sent < 0) {
            std::cerr << "sendto failed: " << socket_last_error() << std::endl;
            return false;
        }
        sentOk = true;
#endif

        if (!sentOk) {
            return false;
        }
        bytesSentForFrame += static_cast<uint64_t>(packet.size());
    }

    m_totalBytesSent.fetch_add(bytesSentForFrame);
    m_totalPacketsSent.fetch_add(static_cast<uint64_t>(chunkCount));
    m_totalFramesSent.fetch_add(1);
    return true;
#endif
}

void LowBandwidthStreamer::logAvError(const std::string& prefix, int errnum) const {
    char buffer[AV_ERROR_MAX_STRING_SIZE] = {};
    av_strerror(errnum, buffer, sizeof(buffer));
    std::cerr << prefix << ": " << buffer << std::endl;
}

void LowBandwidthStreamer::cleanup() {
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
    if (m_codecContext) {
        avcodec_free_context(&m_codecContext);
    }
    if (m_cap.isOpened()) {
        m_cap.release();
    }
    if (m_udpSocket != kInvalidSocket) {
        close_socket(m_udpSocket);
        m_udpSocket = kInvalidSocket;
    }
    if (m_serialHandle != kInvalidSerialHandle) {
#ifdef _WIN32
        CloseHandle(m_serialHandle);
#else
        ::close(m_serialHandle);
#endif
        m_serialHandle = kInvalidSerialHandle;
    }
#if LOW_BANDWIDTH_TRANSPORT_MODE == LOW_BANDWIDTH_TRANSPORT_UDP
    if (m_initialized.load()) {
        socket_cleanup();
    }
#endif
    m_initialized = false;
}
