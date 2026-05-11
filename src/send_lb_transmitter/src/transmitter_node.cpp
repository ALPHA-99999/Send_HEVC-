#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cerrno>
#include <cstring>
#include <iomanip>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <stdexcept>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace
{
bool write_all(int fd, const uint8_t * data, size_t size)
{
  if (fd < 0) {
    return false;
  }
  size_t written_total = 0;
  while (written_total < size) {
    const ssize_t written = ::write(fd, data + written_total, size - written_total);
    if (written < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    if (written == 0) {
      return false;
    }
    written_total += static_cast<size_t>(written);
  }
  return true;
}

speed_t baud_to_termios(std::uint32_t baud)
{
  switch (baud) {
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

bool configure_serial_port(int fd, std::uint32_t baud)
{
  termios tty = {};
  if (tcgetattr(fd, &tty) != 0) {
    return false;
  }
  cfmakeraw(&tty);
  const speed_t speed = baud_to_termios(baud);
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
    return false;
  }
  tcflush(fd, TCIOFLUSH);
  return true;
}

enum class TransportMode { Udp, Serial };
enum class SliceMode { Fixed, Mtu, Whole };

constexpr std::size_t kHeaderSize = 8;

void store_u16(std::uint8_t * dst, std::uint16_t v)
{
  dst[0] = static_cast<std::uint8_t>((v >> 8) & 0xFF);
  dst[1] = static_cast<std::uint8_t>(v & 0xFF);
}

void store_u32(std::uint8_t * dst, std::uint32_t v)
{
  dst[0] = static_cast<std::uint8_t>((v >> 24) & 0xFF);
  dst[1] = static_cast<std::uint8_t>((v >> 16) & 0xFF);
  dst[2] = static_cast<std::uint8_t>((v >> 8) & 0xFF);
  dst[3] = static_cast<std::uint8_t>(v & 0xFF);
}

}  // namespace

class HevcTransmitterNode : public rclcpp::Node
{
public:
  HevcTransmitterNode()
  : Node("transmitter_node")
  {
    declare_parameter<std::string>("input_topic", "/send_lb/hevc_stream");
    declare_parameter<std::string>("transport_mode", "udp");
    declare_parameter<std::string>("slice_mode", "fixed");
    declare_parameter<int>("payload_size", 256);
    declare_parameter<int>("udp_mtu_payload", 1200);
    declare_parameter<int>("serial_mtu_payload", 256);
    declare_parameter<std::string>("udp_address", "127.0.0.1");
    declare_parameter<int>("udp_port", 3334);
    declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    declare_parameter<int>("serial_baud_rate", 921600);
    declare_parameter<int>("send_hz", 0);

    get_parameter("input_topic", input_topic_);
    get_parameter("transport_mode", transport_mode_str_);
    get_parameter("slice_mode", slice_mode_str_);
    get_parameter("payload_size", payload_size_);
    get_parameter("udp_mtu_payload", udp_mtu_payload_);
    get_parameter("serial_mtu_payload", serial_mtu_payload_);
    get_parameter("udp_address", udp_address_);
    get_parameter("udp_port", udp_port_);
    get_parameter("serial_port", serial_port_);
    get_parameter("serial_baud_rate", serial_baud_rate_);
    get_parameter("send_hz", send_hz_);

    transport_mode_ = (transport_mode_str_ == "serial") ? TransportMode::Serial : TransportMode::Udp;
    if (slice_mode_str_ == "mtu") {
      slice_mode_ = SliceMode::Mtu;
    } else if (slice_mode_str_ == "whole") {
      slice_mode_ = SliceMode::Whole;
    } else {
      slice_mode_ = SliceMode::Fixed;
    }

    init_transport();
    subscription_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&HevcTransmitterNode::packet_callback, this, std::placeholders::_1));
    stats_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&HevcTransmitterNode::print_stats, this));

    RCLCPP_INFO(get_logger(), "Transmitter subscribed to %s mode=%s slice=%s",
                input_topic_.c_str(), transport_mode_str_.c_str(), slice_mode_str_.c_str());
  }

  ~HevcTransmitterNode() override
  {
    if (udp_socket_ >= 0) {
      ::close(udp_socket_);
    }
    if (serial_fd_ >= 0) {
      ::close(serial_fd_);
    }
  }

private:
  void init_transport()
  {
    if (transport_mode_ == TransportMode::Serial) {
      serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
      if (serial_fd_ < 0) {
        throw std::runtime_error("Failed to open serial port: " + serial_port_);
      }
      if (!configure_serial_port(serial_fd_, static_cast<std::uint32_t>(serial_baud_rate_))) {
        throw std::runtime_error("Failed to configure serial port");
      }
      return;
    }

    udp_socket_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udp_socket_ < 0) {
      throw std::runtime_error("Failed to create UDP socket");
    }
    std::memset(&udp_addr_, 0, sizeof(udp_addr_));
    udp_addr_.sin_family = AF_INET;
    udp_addr_.sin_port = htons(static_cast<std::uint16_t>(udp_port_));
    if (inet_pton(AF_INET, udp_address_.c_str(), &udp_addr_.sin_addr) != 1) {
      throw std::runtime_error("Invalid UDP address: " + udp_address_);
    }
  }

  std::size_t choose_payload_size(std::size_t frame_size) const
  {
    switch (slice_mode_) {
      case SliceMode::Whole:
        return frame_size;
      case SliceMode::Mtu:
        return transport_mode_ == TransportMode::Serial ? static_cast<std::size_t>(std::max(1, serial_mtu_payload_))
                                                        : static_cast<std::size_t>(std::max(1, udp_mtu_payload_));
      case SliceMode::Fixed:
      default:
        return static_cast<std::size_t>(std::max(1, payload_size_));
    }
  }

  bool send_packet(const std::vector<std::uint8_t> & packet)
  {
    if (transport_mode_ == TransportMode::Serial) {
      return write_all(serial_fd_, packet.data(), packet.size());
    }
    const int sent = ::sendto(udp_socket_,
                              reinterpret_cast<const char *>(packet.data()),
                              static_cast<int>(packet.size()),
                              0,
                              reinterpret_cast<sockaddr *>(&udp_addr_),
                              sizeof(udp_addr_));
    return sent >= 0;
  }

  void packet_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    if (!msg || msg->data.empty()) {
      return;
    }

    const std::size_t frame_size = msg->data.size();
    const std::size_t payload_size = choose_payload_size(frame_size);
    const std::size_t chunk_count = (frame_size + payload_size - 1) / payload_size;

    for (std::size_t chunk_index = 0; chunk_index < chunk_count; ++chunk_index) {
      const std::size_t offset = chunk_index * payload_size;
      const std::size_t chunk_size = std::min(payload_size, frame_size - offset);

      std::vector<std::uint8_t> packet(kHeaderSize + chunk_size);
      store_u16(packet.data() + 0, frame_seq_);
      store_u16(packet.data() + 2, static_cast<std::uint16_t>(chunk_index));
      store_u32(packet.data() + 4, static_cast<std::uint32_t>(frame_size));
      std::memcpy(packet.data() + kHeaderSize, msg->data.data() + offset, chunk_size);

      if (!send_packet(packet)) {
        RCLCPP_ERROR(get_logger(), "Failed to send packet");
        return;
      }

      bytes_sent_ += packet.size();
      packets_sent_ += 1;
    }

    frames_sent_ += 1;
    frame_seq_ += 1;

    if (send_hz_ > 0) {
      const auto interval = std::chrono::microseconds(1000000 / std::max(1, send_hz_));
      std::this_thread::sleep_for(interval);
    }
  }

  void print_stats()
  {
    const auto bytes = bytes_sent_.exchange(0);
    const auto packets = packets_sent_.exchange(0);
    const auto frames = frames_sent_.exchange(0);
    RCLCPP_INFO(get_logger(), "TX %zu bytes/s | packets/s %zu | frames/s %zu",
                bytes, packets, frames);
  }

  std::string input_topic_;
  std::string transport_mode_str_{"udp"};
  std::string slice_mode_str_{"fixed"};
  int payload_size_{256};
  int udp_mtu_payload_{1200};
  int serial_mtu_payload_{256};
  std::string udp_address_{"127.0.0.1"};
  int udp_port_{3334};
  std::string serial_port_{"/dev/ttyUSB0"};
  int serial_baud_rate_{921600};
  int send_hz_{0};

  TransportMode transport_mode_{TransportMode::Udp};
  SliceMode slice_mode_{SliceMode::Fixed};

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  int udp_socket_{-1};
  sockaddr_in udp_addr_{};
  int serial_fd_{-1};

  std::atomic<std::size_t> bytes_sent_{0};
  std::atomic<std::size_t> packets_sent_{0};
  std::atomic<std::size_t> frames_sent_{0};
  std::uint16_t frame_seq_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HevcTransmitterNode>());
  rclcpp::shutdown();
  return 0;
}
