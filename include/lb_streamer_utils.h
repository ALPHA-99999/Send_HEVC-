#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <opencv2/opencv.hpp>

#include <libavcodec/avcodec.h>
#include <termios.h>

namespace lb_streamer_detail {

constexpr std::size_t kWirePacketSize = 300;
constexpr std::size_t kStreamHeaderSize = 16;
constexpr std::size_t kStreamPayloadSize = kWirePacketSize - kStreamHeaderSize;

constexpr std::uint32_t kStreamMagic = 0x4C425331; // "LBS1"
constexpr std::uint8_t kStreamVersion = 1;

constexpr std::uint8_t kStreamFlagKeyframe = 1u << 0;
constexpr std::uint8_t kStreamFlagEndOfStream = 1u << 1;

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
