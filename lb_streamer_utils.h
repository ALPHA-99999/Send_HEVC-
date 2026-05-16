#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <opencv2/opencv.hpp>

#include <libavcodec/avcodec.h>
#include <termios.h>

namespace lb_streamer_detail {

constexpr int kHeaderSize = 8;

cv::Mat fitIntoCanvas(const cv::Mat& image, const cv::Size& targetSize, double& scale, cv::Point& offset);
bool writeSerialAll(int serialHandle, const uint8_t* data, size_t size);
speed_t baudToTermios(std::uint32_t baudRate);
bool configureLinuxSerialPort(int fd, std::uint32_t baudRate);
AVPixelFormat chooseEncoderPixelFormat(const std::string& codecName);
std::string normalizeEncoderName(const std::string& codecName);
bool isHardwareVaapiEncoder(const std::string& codecName);
bool tryOpenCameraDevice(cv::VideoCapture& cap, int cameraIndex);
void applyCameraParams(cv::VideoCapture& cap, int width, int height, int fps);

} // namespace lb_streamer_detail
