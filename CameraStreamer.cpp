#include "CameraStreamer.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <vector>

int cnt = 0;
namespace {
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
} // namespace

CameraStreamer::CameraStreamer(int cameraIndex, int width, int height,
                               int fps, const char* udpAddress, int udpPort, const char* codec_name)
    : m_cameraIndex(cameraIndex)
    , m_width(width)
    , m_height(height)
    , m_fps(fps)
    , m_udpAddress(udpAddress)
    , m_udpPort(udpPort)
    , m_codecName(codec_name ? codec_name : "libx265")
    , m_isStreaming(false)
    , m_frameCounter(0)
    , m_codec(nullptr)
    , m_codecContext(nullptr)
    , m_yuvFrame(nullptr)
    , m_packet(nullptr)
    , m_swsContext(nullptr)
    , m_udpSocket(kInvalidSocket) {
}

CameraStreamer::~CameraStreamer() {
    stopStreaming();
    cleanup();
}

bool CameraStreamer::initialize() {
    if (!socket_init()) {
        std::cerr << "Socket runtime initialization failed" << std::endl;
        return false;
    }

    if (!initCamera()) {
        std::cerr << "Failed to initialize camera" << std::endl;
        return false;
    }

    if (!initFFmpeg()) {
        std::cerr << "Failed to initialize FFmpeg" << std::endl;
        return false;
    }

    if (!initSocket()) {
        std::cerr << "Failed to initialize socket" << std::endl;
        return false;
    }

    return true;
}

bool CameraStreamer::initCamera() {
    if (!tryOpenCameraDevice(m_cap, m_cameraIndex)) {
        for (int idx = 0; idx <= 5; ++idx) {
            if (idx == m_cameraIndex) {
                continue;
            }
            if (tryOpenCameraDevice(m_cap, idx)) {
                m_cameraIndex = idx;
                break;
            }
        }
    }

    if (!m_cap.isOpened()) {
        std::cerr << "Failed to open camera. Tried indexes 0-5." << std::endl;
        return false;
    }

    applyCameraParams(m_cap, m_width, m_height, m_fps);

    std::cout << "Camera initialized: "
              << m_cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x" << m_cap.get(cv::CAP_PROP_FRAME_HEIGHT)
              << " @ " << m_cap.get(cv::CAP_PROP_FPS) << "fps"
              << " (cap index " << m_cameraIndex << ")" << std::endl;
    return true;
}

bool CameraStreamer::initFFmpeg() {
    if (m_codecName == "libx265") {
        m_codec = avcodec_find_encoder_by_name("libx265");
    } else if (m_codecName == "hevc_qsv") {
        m_codec = avcodec_find_encoder_by_name("hevc_qsv");
    }

    if (!m_codec) {
        std::cerr << "HEVC codec not found: " << m_codecName << std::endl;
        return false;
    }

    m_codecContext = avcodec_alloc_context3(m_codec);
    if (!m_codecContext) {
        std::cerr << "Could not allocate video codec context" << std::endl;
        return false;
    }

    m_codecContext->bit_rate = 2500000;
    m_codecContext->width = m_width;
    m_codecContext->height = m_height;
    m_codecContext->time_base = {1, m_fps};
    m_codecContext->framerate = {m_fps, 1};
    m_codecContext->gop_size = 10;
    m_codecContext->max_b_frames = 0;
    if (std::strcmp(m_codec->name, "libx265") == 0) {
        m_codecContext->pix_fmt = AV_PIX_FMT_YUV420P;
    } else if (std::strcmp(m_codec->name, "hevc_qsv") == 0) {
        m_codecContext->pix_fmt = AV_PIX_FMT_YUYV422;
    }

    av_opt_set(m_codecContext->priv_data, "preset", "fast", 0);
    av_opt_set(m_codecContext->priv_data, "tune", "zerolatency", 0);

    if (avcodec_open2(m_codecContext, m_codec, nullptr) < 0) {
        std::cerr << "Could not open codec" << std::endl;
        return false;
    }

    m_yuvFrame = av_frame_alloc();
    if (!m_yuvFrame) {
        std::cerr << "Could not allocate video frame" << std::endl;
        return false;
    }
    m_yuvFrame->format = m_codecContext->pix_fmt;
    m_yuvFrame->width = m_codecContext->width;
    m_yuvFrame->height = m_codecContext->height;

    if (av_frame_get_buffer(m_yuvFrame, 0) < 0) {
        std::cerr << "Could not allocate the video frame data" << std::endl;
        return false;
    }

    m_packet = av_packet_alloc();
    if (!m_packet) {
        std::cerr << "Could not allocate packet" << std::endl;
        return false;
    }

    m_swsContext = sws_getContext(
        m_width, m_height, AV_PIX_FMT_BGR24,
        m_width, m_height, m_codecContext->pix_fmt,
        SWS_BILINEAR, nullptr, nullptr, nullptr);

    if (!m_swsContext) {
        std::cerr << "Could not initialize sws context" << std::endl;
        return false;
    }

    std::cout << "FFmpeg initialized successfully" << std::endl;
    return true;
}

bool CameraStreamer::initSocket() {
    m_udpSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (m_udpSocket == kInvalidSocket) {
        std::cerr << "Socket creation failed: " << socket_last_error() << std::endl;
        return false;
    }

    int broadcast = 1;
    setsockopt(m_udpSocket, SOL_SOCKET, SO_BROADCAST,
               reinterpret_cast<const char*>(&broadcast), sizeof(broadcast));

    std::memset(&m_serverAddr, 0, sizeof(m_serverAddr));
    m_serverAddr.sin_family = AF_INET;
    m_serverAddr.sin_port = htons(static_cast<uint16_t>(m_udpPort));
    inet_pton(AF_INET, m_udpAddress.c_str(), &m_serverAddr.sin_addr);

    std::cout << "Socket initialized: " << m_udpAddress
              << ":" << m_udpPort << std::endl;
    return true;
}

AVFrame* CameraStreamer::convertToYUV(const cv::Mat& rgbFrame) {
    if (rgbFrame.cols != m_width || rgbFrame.rows != m_height) {
        std::cerr << "Frame size mismatch" << std::endl;
        return nullptr;
    }

    uint8_t* srcData[1] = {const_cast<uint8_t*>(rgbFrame.data)};
    int srcLinesize[1] = {static_cast<int>(rgbFrame.step)};

    sws_scale(m_swsContext, srcData, srcLinesize, 0,
              m_height, m_yuvFrame->data, m_yuvFrame->linesize);

    m_yuvFrame->pts = m_frameCounter;
    return m_yuvFrame;
}

void CameraStreamer::sendPacket(const uint8_t* data, int size, uint16_t frameIndex) {
    if (size <= 0 || !data) {
        return;
    }

    const int maxPacketSize = 1400;
    const int headerSize = 8;
    const int payloadSize = maxPacketSize - headerSize;
    const int totalPackets = (size + payloadSize - 1) / payloadSize;

    for (int packetIndex = 0; packetIndex < totalPackets; packetIndex++) {
        int offset = packetIndex * payloadSize;
        int packetSize = std::min(payloadSize, size - offset);

        std::vector<uint8_t> packetData(headerSize + packetSize);
        packetData[0] = (frameIndex >> 8) & 0xFF;
        packetData[1] = frameIndex & 0xFF;
        packetData[2] = (packetIndex >> 8) & 0xFF;
        packetData[3] = packetIndex & 0xFF;
        packetData[4] = (size >> 24) & 0xFF;
        packetData[5] = (size >> 16) & 0xFF;
        packetData[6] = (size >> 8) & 0xFF;
        packetData[7] = size & 0xFF;

        std::memcpy(packetData.data() + headerSize, data + offset, packetSize);

        sendto(m_udpSocket,
               reinterpret_cast<const char*>(packetData.data()),
               static_cast<int>(packetData.size()),
               0,
               reinterpret_cast<sockaddr*>(&m_serverAddr),
               sizeof(m_serverAddr));
    }

    std::cout << "Frame " << frameIndex << " sent: "
              << size << " bytes in " << totalPackets << " packets" << std::endl;
}

bool CameraStreamer::encodeAndSendFrame(const cv::Mat& frame, uint16_t frameIndex) {
    AVFrame* yuvFrame = convertToYUV(frame);
    if (!yuvFrame) {
        return false;
    }

    int ret = avcodec_send_frame(m_codecContext, yuvFrame);
    if (ret < 0) {
        std::cerr << "Error sending frame to encoder" << std::endl;
        return false;
    }

    while (ret >= 0) {
        ret = avcodec_receive_packet(m_codecContext, m_packet);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            break;
        }
        if (ret < 0) {
            std::cerr << "Error during encoding" << std::endl;
            return false;
        }

        sendPacket(m_packet->data, m_packet->size, frameIndex);
        av_packet_unref(m_packet);
    }

    return true;
}

void CameraStreamer::getFrameLoop() {
    std::cout << "Capture thread started" << std::endl;
    int consecutiveFailures = 0;

    while (m_isStreaming.load()) {
        cv::Mat frame;
        if (!m_cap.read(frame) || frame.empty()) {
            ++consecutiveFailures;
            if (consecutiveFailures == 1 || consecutiveFailures % 50 == 0) {
                std::cerr << "Failed to capture frame from cap("
                          << m_cameraIndex << "), retry=" << consecutiveFailures << std::endl;
            }
            if (consecutiveFailures % 100 == 0) {
                m_cap.release();
                if (tryOpenCameraDevice(m_cap, m_cameraIndex)) {
                    applyCameraParams(m_cap, m_width, m_height, m_fps);
                    consecutiveFailures = 0;
                }
            }
            sleep_ms(10);
            continue;
        }
        consecutiveFailures = 0;

        std::lock_guard<std::mutex> lock(m_queueMutex);
        if (m_frameQueue.size() >= 10) {
            m_frameQueue.pop();
        }
        m_frameQueue.push(frame);
    }

    std::cout << "Capture thread stopped" << std::endl;
}

void CameraStreamer::encoderLoop() {
    std::cout << "Encoder thread started" << std::endl;

    while (m_isStreaming.load()) {
        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lock(m_queueMutex);
            if (!m_frameQueue.empty()) {
                frame = m_frameQueue.front();
                m_frameQueue.pop();
            }
        }

        if (frame.empty()) {
            sleep_ms(1);
            continue;
        }

        const uint16_t frameIndex = m_frameCounter.fetch_add(1);
        if (!encodeAndSendFrame(frame, frameIndex)) {
            std::cerr << "Failed to encode frame" << std::endl;
        }

        ++cnt;
        cv::imshow("Camera Stream", frame);
        if (cv::waitKey(1) == 27) {
            m_isStreaming = false;
            break;
        }
    }

    std::cout << "Encoder thread stopped" << std::endl;
}

void CameraStreamer::startStreaming() {
    if (m_isStreaming.load()) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(m_queueMutex);
        while (!m_frameQueue.empty()) {
            m_frameQueue.pop();
        }
    }

    m_isStreaming = true;
    m_frameCounter = 0;

    m_getFrameThread = std::thread(&CameraStreamer::getFrameLoop, this);
    m_encoderThread = std::thread(&CameraStreamer::encoderLoop, this);
}

void CameraStreamer::stopStreaming() {
    if (!m_isStreaming.exchange(false)) {
        return;
    }

    if (m_getFrameThread.joinable()) {
        m_getFrameThread.join();
    }
    if (m_encoderThread.joinable()) {
        m_encoderThread.join();
    }

    cv::destroyAllWindows();
}

void CameraStreamer::cleanup() {
    if (m_swsContext) {
        sws_freeContext(m_swsContext);
        m_swsContext = nullptr;
    }

    if (m_packet) {
        av_packet_free(&m_packet);
        m_packet = nullptr;
    }

    if (m_yuvFrame) {
        av_frame_free(&m_yuvFrame);
        m_yuvFrame = nullptr;
    }

    if (m_codecContext) {
        avcodec_free_context(&m_codecContext);
        m_codecContext = nullptr;
    }

    if (m_cap.isOpened()) {
        m_cap.release();
    }

    if (m_udpSocket != kInvalidSocket) {
        close_socket(m_udpSocket);
        m_udpSocket = kInvalidSocket;
    }

    socket_cleanup();
}
