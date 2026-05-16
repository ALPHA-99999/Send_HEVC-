#include "hikrobot_camera.hpp"

#ifdef HAVE_HIKROBOT_SDK

#include <cmath>
#include <cctype>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <dlfcn.h>

using namespace std::chrono_literals;

namespace
{
std::string sdk_error_hint(unsigned int ret)
{
  if (ret == MV_E_RESOURCE) {
    return " (likely USB permission or device access issue; check udev rules or run as root)";
  }
  if (ret == MV_E_USB_DRIVER) {
    return " (USB driver/runtime mismatch)";
  }
  return "";
}

const char * pixel_type_name(unsigned int pixel_type)
{
  switch (static_cast<MvGvspPixelType>(pixel_type)) {
    case PixelType_Gvsp_Mono8: return "Mono8";
    case PixelType_Gvsp_HB_Mono8: return "HB_Mono8";
    case PixelType_Gvsp_BGR8_Packed: return "BGR8_Packed";
    case PixelType_Gvsp_HB_BGR8_Packed: return "HB_BGR8_Packed";
    case PixelType_Gvsp_RGB8_Packed: return "RGB8_Packed";
    case PixelType_Gvsp_HB_RGB8_Packed: return "HB_RGB8_Packed";
    case PixelType_Gvsp_BayerGR8: return "BayerGR8";
    case PixelType_Gvsp_BayerRG8: return "BayerRG8";
    case PixelType_Gvsp_BayerGB8: return "BayerGB8";
    case PixelType_Gvsp_BayerBG8: return "BayerBG8";
    case PixelType_Gvsp_HB_BayerGR8: return "HB_BayerGR8";
    case PixelType_Gvsp_HB_BayerRG8: return "HB_BayerRG8";
    case PixelType_Gvsp_HB_BayerGB8: return "HB_BayerGB8";
    case PixelType_Gvsp_HB_BayerBG8: return "HB_BayerBG8";
    case PixelType_Gvsp_YUV422_Packed: return "YUV422_Packed";
    case PixelType_Gvsp_YUV422_YUYV_Packed: return "YUV422_YUYV_Packed";
    case PixelType_Gvsp_HB_YUV422_Packed: return "HB_YUV422_Packed";
    case PixelType_Gvsp_HB_YUV422_YUYV_Packed: return "HB_YUV422_YUYV_Packed";
    case PixelType_Gvsp_YCBCR422_8: return "YCBCR422_8";
    case PixelType_Gvsp_YCBCR422_8_CBYCRY: return "YCBCR422_8_CBYCRY";
    case PixelType_Gvsp_YCBCR601_422_8: return "YCBCR601_422_8";
    case PixelType_Gvsp_YCBCR601_422_8_CBYCRY: return "YCBCR601_422_8_CBYCRY";
    case PixelType_Gvsp_YCBCR709_422_8: return "YCBCR709_422_8";
    case PixelType_Gvsp_YCBCR709_422_8_CBYCRY: return "YCBCR709_422_8_CBYCRY";
    default: return "Unknown";
  }
}

void log_enum_value(void * handle, const char * key)
{
  MVCC_ENUMVALUE value{};
  const auto ret = MV_CC_GetEnumValue(handle, key, &value);
  if (ret != MV_OK) {
    std::cerr << "[warn] MV_CC_GetEnumValue(" << key << ") failed: " << ret << "\n";
    return;
  }

  std::cerr << "[info] HikRobot " << key << ": 0x"
            << std::hex << value.nCurValue << std::dec
            << " (" << pixel_type_name(value.nCurValue) << ")\n";
}

void log_frame_rate(void * handle)
{
  MVCC_FLOATVALUE value{};
  const auto ret = MV_CC_GetFrameRate(handle, &value);
  if (ret != MV_OK) {
    std::cerr << "[warn] MV_CC_GetFrameRate failed: " << ret << "\n";
    return;
  }

  std::cerr << "[info] HikRobot FrameRate: current=" << value.fCurValue
            << " min=" << value.fMin
            << " max=" << value.fMax << "\n";
}
}  // namespace

HikRobotCamera::HikRobotCamera(double exposure_ms,
                               double gain,
                               std::string vid_pid,
                               std::string record_dir,
                               double record_fps,
                               bool record_raw_video)
: exposure_us_(exposure_ms * 1000.0),
  gain_(gain),
  vid_pid_(std::move(vid_pid)),
  record_dir_(std::move(record_dir)),
  requested_record_fps_(record_fps),
  record_fps_(record_fps),
  record_raw_video_(record_raw_video),
  vid_(-1),
  pid_(-1),
  handle_(nullptr),
  handle_created_(false),
  device_opened_(false),
  grabbing_started_(false),
  quit_(false),
  queue_limit_(1),
  raw_writer_opened_(false),
  raw_pending_have_first_timestamp_(false),
  libusb_ready_(false)
{
  parse_vid_pid();
  if (libusb_init(nullptr) == 0) {
    libusb_ready_ = true;
  } else {
    std::cerr << "[warn] libusb init failed, USB reset will be skipped.\n";
  }

  std::filesystem::create_directories("mvs_log");
  if (record_raw_video_) {
    std::filesystem::create_directories(record_dir_);
  }
  MV_CC_SetSDKLogPath("mvs_log");

  check_runtime_dependencies();
  open();
  capture_thread_ = std::thread(&HikRobotCamera::capture_loop, this);
}

HikRobotCamera::~HikRobotCamera()
{
  close();
}

void HikRobotCamera::parse_vid_pid()
{
  if (vid_pid_.empty()) return;

  const auto pos = vid_pid_.find(':');
  if (pos == std::string::npos) {
    throw std::runtime_error("Invalid vid_pid format, expected hex hex separated by ':'");
  }

  try {
    vid_ = std::stoi(vid_pid_.substr(0, pos), nullptr, 16);
    pid_ = std::stoi(vid_pid_.substr(pos + 1), nullptr, 16);
  } catch (const std::exception &) {
    throw std::runtime_error("Invalid vid_pid value, expected hex numbers like 0x2bdf:0x1234");
  }
}

bool HikRobotCamera::select_device(MV_CC_DEVICE_INFO_LIST & device_list, MV_CC_DEVICE_INFO *& device) const
{
  device = nullptr;
  for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
    auto * info = device_list.pDeviceInfo[i];
    if (!info) continue;
    if (info->nTLayerType != MV_USB_DEVICE) continue;

    if (vid_ >= 0 && pid_ >= 0) {
      const auto & usb = info->SpecialInfo.stUsb3VInfo;
      if (static_cast<int>(usb.idVendor) != vid_ || static_cast<int>(usb.idProduct) != pid_) {
        continue;
      }
    }

    device = info;
    return true;
  }
  return false;
}

void HikRobotCamera::fail_and_cleanup(const std::string & message)
{
  std::string error = message;
  close();
  throw std::runtime_error(error);
}

void HikRobotCamera::check_runtime_dependencies() const
{
  const char * libs[] = {"libMvUsb3vTL.so", "libMVGigEVisionSDK.so", "libMvCamLVision.so"};
  bool missing = false;
  for (const char * lib : libs) {
    void * handle = dlopen(lib, RTLD_LAZY | RTLD_LOCAL);
    if (handle) {
      dlclose(handle);
      continue;
    }
    missing = true;
    std::cerr << "[warn] missing Hikrobot runtime library: " << lib << " (" << dlerror() << ")\n";
  }
  if (missing) {
    std::cerr << "[warn] The installed SDK looks incomplete. Install the full Hikrobot MVS runtime package, not only libMvCameraControl.so.\n";
  }
}

void HikRobotCamera::open()
{
  MV_CC_DEVICE_INFO_LIST device_list{};
  auto ret = MV_CC_EnumDevicesEx2(MV_USB_DEVICE, &device_list, "Hikrobot", SortMethod_SerialNumber);
  if (ret != MV_OK) {
    std::cerr << "[warn] MV_CC_EnumDevicesEx2(Hikrobot) failed: " << ret << ", falling back to MV_CC_EnumDevices.\n";
    ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  }
  if (ret != MV_OK) {
    fail_and_cleanup("MV_CC_EnumDevices failed: " + std::to_string(ret) + sdk_error_hint(ret));
  }

  if (device_list.nDeviceNum == 0) {
    fail_and_cleanup("No Hikvision USB camera found.");
  }

  MV_CC_DEVICE_INFO * device = nullptr;
  if (!select_device(device_list, device)) {
    fail_and_cleanup("No camera matched the requested vid_pid.");
  }

  ret = MV_CC_CreateHandle(&handle_, device);
  if (ret != MV_OK) {
    fail_and_cleanup("MV_CC_CreateHandle failed: " + std::to_string(ret) + sdk_error_hint(ret));
  }
  handle_created_ = true;

  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) {
    fail_and_cleanup("MV_CC_OpenDevice failed: " + std::to_string(ret) + sdk_error_hint(ret));
  }
  device_opened_ = true;

  set_enum_value("TriggerMode", MV_TRIGGER_MODE_OFF);
  set_enum_value("AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);
  set_enum_value("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_OFF);
  set_enum_value("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
  set_enum_value("GainAuto", MV_GAIN_MODE_OFF);
  set_enum_value("PixelFormat", PixelType_Gvsp_BGR8_Packed);
  set_bool_value("AcquisitionFrameRateEnable", true);
  set_float_value("ExposureTime", exposure_us_);
  set_float_value("Gain", gain_);
  if (requested_record_fps_ > 0.0) {
    set_float_value("AcquisitionFrameRate", requested_record_fps_);
    MV_CC_SetFrameRate(handle_, static_cast<float>(requested_record_fps_));
  }

  log_enum_value(handle_, "TriggerMode");
  log_enum_value(handle_, "PixelFormat");
  log_frame_rate(handle_);

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    fail_and_cleanup("MV_CC_StartGrabbing failed: " + std::to_string(ret) + sdk_error_hint(ret));
  }
  grabbing_started_ = true;

  std::cerr << "[info] HikRobot camera opened successfully.\n";
}

void HikRobotCamera::close()
{
  quit_ = true;
  cv_.notify_all();

  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }

  if (handle_) {
    if (grabbing_started_) {
      MV_CC_StopGrabbing(handle_);
      grabbing_started_ = false;
    }
    if (device_opened_) {
      MV_CC_CloseDevice(handle_);
      device_opened_ = false;
    }
    if (handle_created_) {
      MV_CC_DestroyHandle(handle_);
      handle_created_ = false;
    }
    handle_ = nullptr;
  }

  if (record_raw_video_) {
    flush_pending_record_frames(true);
  }

  if (libusb_ready_) {
    libusb_exit(nullptr);
    libusb_ready_ = false;
  }

  if (raw_writer_opened_) {
    raw_writer_.release();
    raw_writer_opened_ = false;
  }
}

void HikRobotCamera::set_float_value(const char * name, double value)
{
  const auto ret = MV_CC_SetFloatValue(handle_, name, static_cast<float>(value));
  if (ret != MV_OK) {
    std::cerr << "[warn] MV_CC_SetFloatValue(" << name << ") failed: " << ret << "\n";
  }
}

void HikRobotCamera::set_enum_value(const char * name, unsigned int value)
{
  const auto ret = MV_CC_SetEnumValue(handle_, name, value);
  if (ret != MV_OK) {
    std::cerr << "[warn] MV_CC_SetEnumValue(" << name << ") failed: " << ret << "\n";
  }
}

void HikRobotCamera::set_bool_value(const char * name, bool value)
{
  const auto ret = MV_CC_SetBoolValue(handle_, name, value);
  if (ret != MV_OK) {
    std::cerr << "[warn] MV_CC_SetBoolValue(" << name << ") failed: " << ret << "\n";
  }
}

void HikRobotCamera::push_frame(Frame frame)
{
  std::lock_guard<std::mutex> lock(mutex_);
  while (frames_.size() >= queue_limit_) {
    frames_.pop();
  }
  frames_.push(std::move(frame));
  cv_.notify_one();
}

std::string HikRobotCamera::make_record_path() const
{
  const auto now = std::chrono::system_clock::now();
  const auto tt = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  localtime_r(&tt, &tm);

  std::ostringstream oss;
  oss << record_dir_ << "/hikrobot_raw_"
      << std::put_time(&tm, "%Y%m%d_%H%M%S")
      << ".avi";
  return oss.str();
}

double HikRobotCamera::estimate_record_fps() const
{
  if (raw_pending_frames_.size() < 2 || !raw_pending_have_first_timestamp_) {
    return 0.0;
  }

  const auto & first = raw_pending_frames_.front();
  const auto & last = raw_pending_frames_.back();
  const auto elapsed = std::chrono::duration<double>(last.timestamp - first.timestamp).count();
  if (elapsed <= 0.0) {
    return 0.0;
  }

  const double fps = static_cast<double>(raw_pending_frames_.size() - 1) / elapsed;
  if (!std::isfinite(fps) || fps <= 0.0) {
    return 0.0;
  }
  return fps;
}

bool HikRobotCamera::maybe_open_record_writer(bool force_open)
{
  if (!record_raw_video_ || raw_writer_opened_ || raw_pending_frames_.empty()) {
    return raw_writer_opened_;
  }

  const auto now = std::chrono::steady_clock::now();
  const bool enough_frames = raw_pending_frames_.size() >= 10;
  const bool enough_time = raw_pending_have_first_timestamp_ &&
                          std::chrono::duration<double>(now - raw_pending_first_timestamp_).count() >= 1.0;
  const bool should_open_now = force_open || requested_record_fps_ > 0.0 || enough_frames || enough_time;
  if (!should_open_now) {
    return false;
  }

  double fps = requested_record_fps_;
  if (fps <= 0.0) {
    fps = estimate_record_fps();
  }
  if (fps <= 0.0) {
    if (!force_open) {
      return false;
    }
    fps = 27.0;
  }

  raw_video_path_ = make_record_path();
  const int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  const cv::Size frame_size = raw_pending_frames_.front().img.size();
  if (!raw_writer_.open(raw_video_path_, fourcc, fps, frame_size, true)) {
    std::cerr << "[warn] Failed to open raw video writer: " << raw_video_path_ << "\n";
    raw_video_path_.clear();
    return false;
  }

  record_fps_ = fps;
  raw_writer_opened_ = true;
  std::cerr << "[info] HikRobot raw video recording to: " << raw_video_path_ << "\n";

  for (const auto & pending : raw_pending_frames_) {
    raw_writer_.write(pending.img);
  }
  raw_pending_frames_.clear();
  raw_pending_have_first_timestamp_ = false;
  return true;
}

void HikRobotCamera::flush_pending_record_frames(bool force_open)
{
  if (!record_raw_video_ || raw_writer_opened_ || raw_pending_frames_.empty()) {
    return;
  }
  maybe_open_record_writer(force_open);
}

void HikRobotCamera::record_raw_frame(const cv::Mat& frame, std::chrono::steady_clock::time_point timestamp)
{
  if (!record_raw_video_ || frame.empty()) {
    return;
  }

  if (raw_writer_opened_) {
    raw_writer_.write(frame);
    return;
  }

  if (!raw_pending_have_first_timestamp_) {
    raw_pending_first_timestamp_ = timestamp;
    raw_pending_have_first_timestamp_ = true;
  }

  raw_pending_frames_.push_back(Frame{frame.clone(), timestamp});
  maybe_open_record_writer(false);
}

void HikRobotCamera::capture_loop()
{
  std::cerr << "[info] HikRobot capture loop started.\n";

  while (!quit_) {
    MV_FRAME_OUT raw{};
    const auto ret = MV_CC_GetImageBuffer(handle_, &raw, 100);
    if (ret != MV_OK) {
      if (!quit_) {
        std::cerr << "[warn] MV_CC_GetImageBuffer failed: " << ret << "\n";
      }
      continue;
    }

    const auto timestamp = std::chrono::steady_clock::now();
    const auto & frame_info = raw.stFrameInfo;
    const auto pixel_type = frame_info.enPixelType;

    cv::Mat src;
    cv::Mat dst;

    if (pixel_type == PixelType_Gvsp_BGR8_Packed || pixel_type == PixelType_Gvsp_HB_BGR8_Packed) {
      src = cv::Mat(frame_info.nHeight, frame_info.nWidth, CV_8UC3, raw.pBufAddr);
      dst = src.clone();
    } else if (pixel_type == PixelType_Gvsp_RGB8_Packed || pixel_type == PixelType_Gvsp_HB_RGB8_Packed) {
      src = cv::Mat(frame_info.nHeight, frame_info.nWidth, CV_8UC3, raw.pBufAddr);
      cv::cvtColor(src, dst, cv::COLOR_RGB2BGR);
    } else if (pixel_type == PixelType_Gvsp_Mono8 || pixel_type == PixelType_Gvsp_HB_Mono8) {
      src = cv::Mat(frame_info.nHeight, frame_info.nWidth, CV_8UC1, raw.pBufAddr);
      cv::cvtColor(src, dst, cv::COLOR_GRAY2BGR);
    } else if (pixel_type == PixelType_Gvsp_YUV422_Packed ||
               pixel_type == PixelType_Gvsp_YUV422_YUYV_Packed ||
               pixel_type == PixelType_Gvsp_HB_YUV422_Packed ||
               pixel_type == PixelType_Gvsp_HB_YUV422_YUYV_Packed ||
               pixel_type == PixelType_Gvsp_YCBCR422_8 ||
               pixel_type == PixelType_Gvsp_YCBCR422_8_CBYCRY ||
               pixel_type == PixelType_Gvsp_YCBCR601_422_8 ||
               pixel_type == PixelType_Gvsp_YCBCR601_422_8_CBYCRY ||
               pixel_type == PixelType_Gvsp_YCBCR709_422_8 ||
               pixel_type == PixelType_Gvsp_YCBCR709_422_8_CBYCRY) {
      src = cv::Mat(frame_info.nHeight, frame_info.nWidth, CV_8UC2, raw.pBufAddr);
      cv::cvtColor(src, dst, cv::COLOR_YUV2BGR_YUY2);
    } else {
      src = cv::Mat(frame_info.nHeight, frame_info.nWidth, CV_8UC1, raw.pBufAddr);
      const static std::unordered_map<MvGvspPixelType, int> kBayerMap = {
        {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2BGR},
        {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2BGR},
        {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2BGR},
        {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2BGR},
        {PixelType_Gvsp_HB_BayerGR8, cv::COLOR_BayerGR2BGR},
        {PixelType_Gvsp_HB_BayerRG8, cv::COLOR_BayerRG2BGR},
        {PixelType_Gvsp_HB_BayerGB8, cv::COLOR_BayerGB2BGR},
        {PixelType_Gvsp_HB_BayerBG8, cv::COLOR_BayerBG2BGR},
      };

      const auto it = kBayerMap.find(pixel_type);
      if (it != kBayerMap.end()) {
        cv::cvtColor(src, dst, it->second);
      } else {
        dst = src.clone();
      }
    }

    record_raw_frame(dst, timestamp);
    push_frame(Frame{std::move(dst), timestamp});

    const auto free_ret = MV_CC_FreeImageBuffer(handle_, &raw);
    if (free_ret != MV_OK && !quit_) {
      std::cerr << "[warn] MV_CC_FreeImageBuffer failed: " << free_ret << "\n";
    }
  }

  std::cerr << "[info] HikRobot capture loop stopped.\n";
}

void HikRobotCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  std::unique_lock<std::mutex> lock(mutex_);
  cv_.wait(lock, [this] { return quit_ || !frames_.empty(); });

  if (frames_.empty()) {
    throw std::runtime_error("Camera stopped.");
  }

  auto frame = std::move(frames_.front());
  frames_.pop();
  lock.unlock();

  img = std::move(frame.img);
  timestamp = frame.timestamp;
}

void HikRobotCamera::reset_usb() const
{
  if (!libusb_ready_ || vid_ < 0 || pid_ < 0) return;

  auto * handle = libusb_open_device_with_vid_pid(nullptr, static_cast<uint16_t>(vid_), static_cast<uint16_t>(pid_));
  if (!handle) {
    std::cerr << "[warn] Unable to open USB device for reset.\n";
    return;
  }

  if (libusb_reset_device(handle) != 0) {
    std::cerr << "[warn] Unable to reset USB device.\n";
  }
  libusb_close(handle);
}

#endif  // HAVE_HIKROBOT_SDK
