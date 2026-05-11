#include "lb_streamer_utils.h"

#include <algorithm>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <iostream>

#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

namespace lb_streamer_detail {

cv::Mat fitIntoCanvas(const cv::Mat& image, const cv::Size& targetSize, double& scale, cv::Point& offset)
{
    // 保持宽高比缩放，不拉伸，空白区域用黑边补齐。
    scale = std::min(static_cast<double>(targetSize.width) / static_cast<double>(image.cols),
                     static_cast<double>(targetSize.height) / static_cast<double>(image.rows));
    const int scaledWidth = std::max(1, static_cast<int>(std::lround(image.cols * scale)));
    const int scaledHeight = std::max(1, static_cast<int>(std::lround(image.rows * scale)));

    cv::Mat resized;
    cv::resize(image, resized, cv::Size(scaledWidth, scaledHeight), 0.0, 0.0, cv::INTER_AREA);

    offset.x = (targetSize.width - scaledWidth) / 2;
    offset.y = (targetSize.height - scaledHeight) / 2;

    cv::Mat canvas(targetSize, image.type(), cv::Scalar::all(0));
    resized.copyTo(canvas(cv::Rect(offset.x, offset.y, scaledWidth, scaledHeight)));
    return canvas;
}

bool writeSerialAll(int serialHandle, const uint8_t* data, size_t size)
{
    // 串口写入可能被中断，所以要循环直到全部写完。
    if (serialHandle < 0) {
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
}

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
    // Linux 串口按 raw 模式配置，减少驱动层干预。
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

AVPixelFormat chooseEncoderPixelFormat(const std::string& codecName)
{
    if (codecName.find("_qsv") != std::string::npos) {
        return AV_PIX_FMT_NV12;
    }
    return AV_PIX_FMT_YUV420P;
}

bool tryOpenCameraDevice(cv::VideoCapture& cap, int cameraIndex)
{
    const int backends[] = {cv::CAP_V4L2, cv::CAP_GSTREAMER, cv::CAP_ANY};
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

} // namespace lb_streamer_detail
