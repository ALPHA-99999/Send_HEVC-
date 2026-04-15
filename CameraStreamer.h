#pragma once

#include "platform_compat.h"

#include <atomic>
#include <mutex>
#include <queue>
#include <thread>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

class CameraStreamer {
public:
    CameraStreamer(int cameraIndex = 0, int width = 1280, int height = 720, 
                   int fps = 40, const char* udpAddress = "127.0.0.1", int udpPort = 3334, const char* codec_name="libx265");
    ~CameraStreamer();
    
    bool initialize();
    void startStreaming();
    void stopStreaming();
    bool isStreaming() const { return m_isStreaming; }

private:
    bool initFFmpeg();
    bool initSocket();
    bool initCamera();
    
    AVFrame* convertToYUV(const cv::Mat& rgbFrame);
    bool encodeAndSendFrame(const cv::Mat& frame, uint16_t frameIndex);
    void sendPacket(const uint8_t* data, int size, uint16_t frameIndex);
    void cleanup();

private:
    // 配置参数
    int m_cameraIndex;
    int m_width;
    int m_height;
    int m_fps;
    std::string m_udpAddress;
    int m_udpPort;
    std::string m_codecName;
    // OpenCV
    cv::VideoCapture m_cap;
    
    // FFmpeg
    const AVCodec* m_codec;
    AVCodecContext* m_codecContext;
    AVFrame* m_yuvFrame;
    AVPacket* m_packet;
    struct SwsContext* m_swsContext;
    
    // 网络
    socket_t m_udpSocket;
    sockaddr_in m_serverAddr;
    
    // 控制
    std::atomic<bool> m_isStreaming;
    std::atomic<uint16_t> m_frameCounter;
    std::thread m_getFrameThread;
    std::thread m_encoderThread;
    std::mutex m_queueMutex;
    std::queue<cv::Mat> m_frameQueue;

    void getFrameLoop();
    void encoderLoop();
};
extern int cnt;
