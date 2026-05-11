#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace
{
enum class SourceType
{
  File,
  Camera,
};

constexpr SourceType kSourceType = SourceType::Camera;
constexpr const char * kVideoPath = "/home/arty/Documents/video/1.avi";
constexpr int kCameraIndex = 0;
constexpr int kCaptureFps = 27;
constexpr int kInputWidth = 1920;
constexpr int kInputHeight = 1080;
constexpr const char * kPublishTopic = "/send_lb/image_preprocessed";
}  // namespace

class CapturePreprocessNode : public rclcpp::Node
{
public:
  CapturePreprocessNode()
  : Node("capture_preprocess_node")
  {
    init_source();
    publisher_ = create_publisher<sensor_msgs::msg::Image>(kPublishTopic, rclcpp::SensorDataQoS());
    running_.store(true);
    worker_ = std::thread(&CapturePreprocessNode::run, this);
    RCLCPP_INFO(get_logger(), "Capture node started, publishing: %s", kPublishTopic);
  }

  ~CapturePreprocessNode() override
  {
    running_.store(false);
    if (worker_.joinable()) {
      worker_.join();
    }

    if (cap_.isOpened()) {
      cap_.release();
    }
  }

private:
  void init_source()
  {
    if (kSourceType == SourceType::File) {
      cap_.open(kVideoPath);
      if (!cap_.isOpened()) {
        throw std::runtime_error(std::string("Failed to open video file: ") + kVideoPath);
      }
      if (kCaptureFps > 0) {
        cap_.set(cv::CAP_PROP_FPS, kCaptureFps);
      }
      return;
    }

    if (kSourceType == SourceType::Camera) {
      if (!cap_.open(kCameraIndex)) {
        throw std::runtime_error("Failed to open camera device");
      }
      // cap_.set(cv::CAP_PROP_FRAME_WIDTH, kInputWidth);
      // cap_.set(cv::CAP_PROP_FRAME_HEIGHT, kInputHeight);
      // cap_.set(cv::CAP_PROP_FPS, kCaptureFps);
      // cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
      return;
    }
  }

  bool read_frame(cv::Mat & frame, rclcpp::Time & stamp)
  {
    if (!cap_.read(frame) || frame.empty()) {
      return false;
    }
    stamp = now();
    return true;
  }

  void publish_frame(const cv::Mat & frame, const rclcpp::Time & stamp)
  {
    cv_bridge::CvImage msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "send_lb";
    msg.encoding = "bgr8";
    msg.image = frame;
    auto image_msg = msg.toImageMsg();
    publisher_->publish(*image_msg);
    RCLCPP_INFO(get_logger(), "Frame published");
  }

  void run()
  {
    while (rclcpp::ok() && running_.load()) {
      cv::Mat frame;
      rclcpp::Time stamp = now();
      if (!read_frame(frame, stamp)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      publish_frame(frame, stamp);
    }
  }

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> publisher_;
  std::atomic<bool> running_{false};
  std::thread worker_;
  cv::VideoCapture cap_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CapturePreprocessNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
