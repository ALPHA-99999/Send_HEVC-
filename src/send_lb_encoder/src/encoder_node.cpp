#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <stdexcept>

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#include <libavutil/error.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

namespace
{
constexpr const char * kInputTopic = "/send_lb/image_preprocessed";
constexpr const char * kOutputTopic = "/send_lb/hevc_stream";
constexpr const char * kCodecName = "libx265";
constexpr int kFps = 27;
constexpr int kBitrateKbps = 90;
constexpr int kGopSize = 120;
constexpr int kMaxBFrames = 2;
constexpr const char * kPreset = "slow";
constexpr const char * kTune = "";

AVPixelFormat choose_pixel_format(const std::string & codec_name)
{
  if (codec_name.find("_qsv") != std::string::npos) {
    return AV_PIX_FMT_NV12;
  }
  return AV_PIX_FMT_YUV420P;
}

}  // namespace

class HevcEncoderNode : public rclcpp::Node
{
public:
  HevcEncoderNode()
  : Node("encoder_node")
  {
    publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(kOutputTopic, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      kInputTopic, rclcpp::SensorDataQoS(),
      std::bind(&HevcEncoderNode::image_callback, this, std::placeholders::_1));
    stats_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&HevcEncoderNode::print_stats, this));

    RCLCPP_INFO(get_logger(), "Encoder node subscribed to %s and publishing to %s",
                kInputTopic, kOutputTopic);
  }

  ~HevcEncoderNode() override
  {
    cleanup_encoder();
  }

private:
  void cleanup_encoder()
  {
    if (sws_context_) {
      sws_freeContext(sws_context_);
      sws_context_ = nullptr;
    }
    if (packet_) {
      av_packet_free(&packet_);
    }
    if (frame_) {
      av_frame_free(&frame_);
    }
    if (codec_context_) {
      avcodec_free_context(&codec_context_);
    }
    encoder_ready_ = false;
  }

  bool init_encoder(int width, int height)
  {
    cleanup_encoder();

    codec_ = avcodec_find_encoder_by_name(kCodecName);
    if (!codec_) {
      RCLCPP_ERROR(get_logger(), "Encoder not found: %s", kCodecName);
      return false;
    }

    codec_context_ = avcodec_alloc_context3(codec_);
    if (!codec_context_) {
      RCLCPP_ERROR(get_logger(), "Could not allocate codec context");
      return false;
    }

    codec_context_->bit_rate = static_cast<int64_t>(kBitrateKbps) * 1000;
    codec_context_->rc_min_rate = codec_context_->bit_rate;
    codec_context_->rc_max_rate = codec_context_->bit_rate;
    codec_context_->rc_buffer_size = codec_context_->bit_rate;
    codec_context_->width = width;
    codec_context_->height = height;
    codec_context_->time_base = AVRational{1, std::max(1, kFps)};
    codec_context_->framerate = AVRational{std::max(1, kFps), 1};
    codec_context_->gop_size = std::max(1, kGopSize);
    codec_context_->max_b_frames = std::max(0, kMaxBFrames);
    codec_context_->pix_fmt = choose_pixel_format(kCodecName);

    av_opt_set(codec_context_->priv_data, "preset", kPreset, 0);
    if (*kTune != '\0') {
      av_opt_set(codec_context_->priv_data, "tune", kTune, 0);
    }
    av_opt_set(codec_context_->priv_data, "repeat-headers", "1", 0);

    if (std::string(kCodecName).find("x265") != std::string::npos) {
      std::string x265_params = "keyint=" + std::to_string(codec_context_->gop_size) +
        ":min-keyint=" + std::to_string(codec_context_->gop_size) +
        ":bframes=" + std::to_string(codec_context_->max_b_frames) +
        ":vbv-maxrate=" + std::to_string(kBitrateKbps) +
        ":vbv-bufsize=" + std::to_string(kBitrateKbps) +
        ":aq-mode=2:aq-strength=1.2";
      av_opt_set(codec_context_->priv_data, "x265-params", x265_params.c_str(), 0);
    }

    int ret = avcodec_open2(codec_context_, codec_, nullptr);
    if (ret < 0) {
      log_av_error("Could not open codec", ret);
      cleanup_encoder();
      return false;
    }

    frame_ = av_frame_alloc();
    packet_ = av_packet_alloc();
    if (!frame_ || !packet_) {
      RCLCPP_ERROR(get_logger(), "Could not allocate frame or packet");
      cleanup_encoder();
      return false;
    }

    frame_->format = codec_context_->pix_fmt;
    frame_->width = codec_context_->width;
    frame_->height = codec_context_->height;
    ret = av_frame_get_buffer(frame_, 32);
    if (ret < 0) {
      log_av_error("Could not allocate frame buffer", ret);
      cleanup_encoder();
      return false;
    }

    sws_context_ = sws_getContext(width, height, AV_PIX_FMT_BGR24,
                                  width, height, codec_context_->pix_fmt,
                                  SWS_BILINEAR, nullptr, nullptr, nullptr);
    if (!sws_context_) {
      RCLCPP_ERROR(get_logger(), "Could not initialize sws context");
      cleanup_encoder();
      return false;
    }

    input_width_ = width;
    input_height_ = height;
    encoder_ready_ = true;
    return true;
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge conversion failed: %s", e.what());
      return;
    }

    const cv::Mat & image = cv_ptr->image;
    if (image.empty()) {
      return;
    }

    std::lock_guard<std::mutex> lock(encoder_mutex_);
    if (!encoder_ready_ || image.cols != input_width_ || image.rows != input_height_) {
      if (!init_encoder(image.cols, image.rows)) {
        return;
      }
    }

    int ret = av_frame_make_writable(frame_);
    if (ret < 0) {
      log_av_error("Frame not writable", ret);
      return;
    }

    uint8_t * src_slice[] = {const_cast<uint8_t *>(image.data)};
    int src_stride[] = {static_cast<int>(image.step)};
    sws_scale(sws_context_, src_slice, src_stride, 0, image.rows, frame_->data, frame_->linesize);
    frame_->pts = frame_index_++;

    ret = avcodec_send_frame(codec_context_, frame_);
    if (ret < 0) {
      log_av_error("Error sending frame to encoder", ret);
      return;
    }

    std::size_t packet_count = 0;
    while (ret >= 0) {
      ret = avcodec_receive_packet(codec_context_, packet_);
      if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
        break;
      }
      if (ret < 0) {
        log_av_error("Error during encoding", ret);
        return;
      }

      sensor_msgs::msg::CompressedImage out;
      out.header = msg->header;
      out.format = "hevc";
      out.data.assign(packet_->data, packet_->data + packet_->size);
      publisher_->publish(out);
      av_packet_unref(packet_);
      ++packet_count;
      {
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        encoded_packets_this_window_ += 1;
        encoded_bytes_this_window_ += static_cast<std::uint64_t>(out.data.size());
      }
    }

    if (packet_count > 0) {
      std::lock_guard<std::mutex> stats_lock(stats_mutex_);
      encoded_frames_this_window_ += 1;
      last_frame_width_ = image.cols;
      last_frame_height_ = image.rows;
    }
  }

  void log_av_error(const std::string & prefix, int errnum) const
  {
    char buffer[AV_ERROR_MAX_STRING_SIZE] = {};
    av_strerror(errnum, buffer, sizeof(buffer));
    RCLCPP_ERROR(get_logger(), "%s: %s", prefix.c_str(), buffer);
  }

  void print_stats()
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    RCLCPP_INFO(
      get_logger(),
      "Encoding stats: frames=%llu packets=%llu bytes=%llu last_frame=%dx%d",
      static_cast<unsigned long long>(encoded_frames_this_window_),
      static_cast<unsigned long long>(encoded_packets_this_window_),
      static_cast<unsigned long long>(encoded_bytes_this_window_),
      last_frame_width_,
      last_frame_height_);
    encoded_frames_this_window_ = 0;
    encoded_packets_this_window_ = 0;
    encoded_bytes_this_window_ = 0;
  }

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>> publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  const AVCodec * codec_{nullptr};
  AVCodecContext * codec_context_{nullptr};
  AVFrame * frame_{nullptr};
  AVPacket * packet_{nullptr};
  SwsContext * sws_context_{nullptr};

  std::mutex encoder_mutex_;
  std::mutex stats_mutex_;
  bool encoder_ready_{false};
  int input_width_{0};
  int input_height_{0};
  int64_t frame_index_{0};
  std::uint64_t encoded_frames_this_window_{0};
  std::uint64_t encoded_packets_this_window_{0};
  std::uint64_t encoded_bytes_this_window_{0};
  int last_frame_width_{0};
  int last_frame_height_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HevcEncoderNode>());
  rclcpp::shutdown();
  return 0;
}
