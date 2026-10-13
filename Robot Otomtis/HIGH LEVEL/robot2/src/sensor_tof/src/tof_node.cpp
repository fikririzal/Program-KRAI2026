#include "sensor_tof/tof_node.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cerrno>
#include <cstring>

namespace sensor_tof
{

constexpr std::array<uint8_t, 3> TofNode::kHeader;

const std::array<std::string, 9> TofNode::kSensorNames = {
  "sensor_mid2",       // id 0
  "sensor_mid1",       // id 1
  "sensor1_depan",     // id 2
  "sensor2_depan",     // id 3
  "sensor4_kanan",     // id 4
  "sensor4_belakang",  // id 5
  "sensor3_belakang",  // id 6
  "sensor3_kiri",      // id 7
  "sensor_mid1_2",     // id 8
};

TofNode::TofNode()
: Node("tof_node")
{
  auto ports = declare_parameter<std::vector<std::string>>(
    "ports", std::vector<std::string>{"/dev/ttyACM0"});
  baudrate_ = declare_parameter<int>("baudrate", 921600);

  for (const auto & port : ports) {
    TofBus bus;
    bus.port = port;
    bus.fd = openPort(port);
    buses_.push_back(std::move(bus));
  }

  tof_array_pub_ = create_publisher<sensor_tof::msg::TofArray>("tof/array", 10);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&TofNode::poll, this));
}

TofNode::~TofNode()
{
  closePorts();
}

int TofNode::openPort(const std::string & port)
{
  int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    RCLCPP_FATAL(get_logger(), "Failed to open %s: %s", port.c_str(), strerror(errno));
    throw std::runtime_error("Could not open TOF serial port " + port);
  }

  termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    RCLCPP_FATAL(get_logger(), "tcgetattr failed on %s: %s", port.c_str(), strerror(errno));
    throw std::runtime_error("tcgetattr failed");
  }

  speed_t spd = B921600;
  if (baudrate_ == 115200)      spd = B115200;
  else if (baudrate_ == 460800) spd = B460800;
  else if (baudrate_ == 230400) spd = B230400;

  cfsetispeed(&tty, spd);
  cfsetospeed(&tty, spd);

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ISIG;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;

  tty.c_cc[VTIME] = 0;
  tty.c_cc[VMIN] = 0;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    RCLCPP_FATAL(get_logger(), "tcsetattr failed on %s: %s", port.c_str(), strerror(errno));
    throw std::runtime_error("tcsetattr failed");
  }

  tcflush(fd, TCIFLUSH);

  RCLCPP_INFO(get_logger(), "Opened TOF serial port %s", port.c_str());
  return fd;
}

void TofNode::closePorts()
{
  for (auto & bus : buses_) {
    if (bus.fd >= 0) {
      close(bus.fd);
      bus.fd = -1;
    }
  }
}

bool TofNode::verifyCheckSum(const uint8_t * data, int len)
{
  unsigned int sum = 0;
  for (int k = 0; k < len - 1; ++k) {
    sum += data[k];
  }
  sum %= 256;
  return sum == data[len - 1];
}

void TofNode::poll()
{
  for (auto & bus : buses_) {
    pollBus(bus);
  }
}

void TofNode::pollBus(TofBus & bus)
{
  int available = 0;
  if (ioctl(bus.fd, FIONREAD, &available) < 0) {
    RCLCPP_ERROR(get_logger(), "ioctl(FIONREAD) failed on %s: %s", bus.port.c_str(), strerror(errno));
    return;
  }

  if (available <= 0) {
    return;
  }

  auto & buffer = bus.buffer;
  size_t old_size = buffer.size();
  buffer.resize(old_size + static_cast<size_t>(available));
  ssize_t n = read(bus.fd, buffer.data() + old_size, static_cast<size_t>(available));
  if (n < 0) {
    RCLCPP_WARN(get_logger(), "Read from %s failed: %s", bus.port.c_str(), strerror(errno));
    buffer.resize(old_size);
    return;
  }
  buffer.resize(old_size + static_cast<size_t>(n));

  while (buffer.size() >= static_cast<size_t>(kFrameLength)) {
    if (buffer[0] != kHeader[0] || buffer[1] != kHeader[1] || buffer[2] != kHeader[2]) {
      buffer.erase(buffer.begin());
      continue;
    }

    if (!verifyCheckSum(buffer.data(), kFrameLength)) {
      buffer.erase(buffer.begin());
      continue;
    }

    const uint8_t * frame = buffer.data();
    uint16_t signal = static_cast<uint16_t>(frame[12]) |
      (static_cast<uint16_t>(frame[13]) << 8);

    if (signal == 0) {
      // RCLCPP_WARN(get_logger(), "TOF out of range on %s!", bus.port.c_str());
      RCLCPP_DEBUG(get_logger(), "TOF out of range on %s!", bus.port.c_str());


    } else {
      uint8_t id = frame[3];
      uint32_t distance_mm = static_cast<uint32_t>(frame[8]) |
        (static_cast<uint32_t>(frame[9]) << 8) |
        (static_cast<uint32_t>(frame[10]) << 16);

      // RCLCPP_INFO(
      //   get_logger(),
      //   "[%s] TOF id=%u time=%ums distance=%umm status=%u signal=%u",
      //   bus.port.c_str(), id, system_time, distance_mm, status, signal);

      if (id < 9) {
        distances_[id] = distance_mm;
      }

      // RCLCPP_INFO(get_logger(),
      //   "depankiri:%u depankanan:%u sampingkiri:%u sampingkanan:%u mid1:%u mid2:%u mid3:%u mid4:%u (mm)",
      //   distances_[0], distances_[1], distances_[6], distances_[7],
      //   distances_[2], distances_[3], distances_[4], distances_[5]);

      sensor_tof::msg::TofArray arr;
      arr.header.stamp = now();
      arr.header.frame_id = "tof";
      arr.sensor_mid2       = distances_[3];
      arr.sensor_mid1       = distances_[2];
      arr.sensor_mid3       = distances_[4];
      arr.sensor_mid4       = distances_[5];
      arr.sensor1_depan     = distances_[0];
      arr.sensor2_depan     = distances_[1];
      arr.sensor1_kiri      = distances_[6];
      arr.sensor2_kanan     = distances_[7];
      arr.sensor_mid1_2     = distances_[8];
 
      tof_array_pub_->publish(arr);

      auto it = range_pubs_.find(id);
      if (it == range_pubs_.end()) {
        std::string topic = "tof/" + kSensorNames[id];
        it = range_pubs_.emplace(
          id, create_publisher<sensor_msgs::msg::Range>(topic, 10)).first;
        RCLCPP_INFO(get_logger(), "Discovered TOF id=%u (%s)", id, topic.c_str());
      }

      sensor_msgs::msg::Range range_msg;
      range_msg.header.stamp = now();
      range_msg.header.frame_id = "tof_" + kSensorNames[id];
      range_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
      range_msg.range = static_cast<float>(distance_mm) / 1000.0f;

      it->second->publish(range_msg);
    }

    buffer.erase(buffer.begin(), buffer.begin() + kFrameLength);
  }

  if (buffer.size() > 4096) {
    RCLCPP_WARN(get_logger(), "TOF buffer overflow on %s, dropping %zu bytes", bus.port.c_str(), buffer.size());
    buffer.clear();
  }
}

}  // namespace sensor_tof
