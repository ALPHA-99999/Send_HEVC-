#pragma once

#ifdef HAVE_HIKROBOT_SDK

#include <atomic>
#include <condition_variable>
#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include <string>
#include <thread>

#include "libusb-1.0/libusb.h"
#include "MvCameraControl.h"

class HikRobotCamera
{
public:
  HikRobotCamera(double exposure_ms = 5.0,
                 double gain = 0.0,
                 std::string vid_pid = "",
                 std::string record_dir = "/home/arty/Documents/video",
                 double record_fps = 27.0,
                 bool record_raw_video = true);
  ~HikRobotCamera();

  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);

  HikRobotCamera(const HikRobotCamera &) = delete;
  HikRobotCamera & operator=(const HikRobotCamera &) = delete;

private:
  struct Frame
  {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
  };

  void open();
  void close();
  void capture_loop();
  void push_frame(Frame frame);
  void check_runtime_dependencies() const;
  void record_raw_frame(const cv::Mat& frame, std::chrono::steady_clock::time_point timestamp);
  bool maybe_open_record_writer(bool force_open = false);
  double estimate_record_fps() const;
  void flush_pending_record_frames(bool force_open);
  std::string make_record_path() const;

  void set_float_value(const char * name, double value);
  void set_enum_value(const char * name, unsigned int value);

  bool select_device(MV_CC_DEVICE_INFO_LIST & device_list, MV_CC_DEVICE_INFO *& device) const;
  void parse_vid_pid();
  void reset_usb() const;
  void fail_and_cleanup(const std::string & message);

  double exposure_us_;
  double gain_;
  std::string vid_pid_;
  std::string record_dir_;
  double requested_record_fps_;
  double record_fps_;
  bool record_raw_video_;
  int vid_;
  int pid_;

  void * handle_;
  bool handle_created_;
  bool device_opened_;
  bool grabbing_started_;

  std::atomic<bool> quit_;
  std::thread capture_thread_;
  cv::VideoWriter raw_writer_;
  bool raw_writer_opened_;
  std::string raw_video_path_;
  std::deque<Frame> raw_pending_frames_;
  std::chrono::steady_clock::time_point raw_pending_first_timestamp_;
  bool raw_pending_have_first_timestamp_;

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::queue<Frame> frames_;
  std::size_t queue_limit_;

  bool libusb_ready_;
};

#endif  // HAVE_HIKROBOT_SDK
