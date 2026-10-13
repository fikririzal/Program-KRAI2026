#include "hardware1_com/acm_node.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace hardware1_com
{

AcmNode::AcmNode()
: Node("acm_node"), fd_(-1)
{
  port_ = declare_parameter<std::string>("port", "/dev/ttyACM0");

  openPort();

  lifter_cmd_sub_ = create_subscription<hardware1_com::msg::LifterCmd>(
    "lifter_cmd", 10,
    std::bind(&AcmNode::lifterCmdCb, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "AcmNode ready — subscribing /lifter_cmd @ %s", port_.c_str());
}

AcmNode::~AcmNode()
{
  closePort();
}

void AcmNode::openPort()
{
  fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    RCLCPP_FATAL(get_logger(), "Failed to open %s: %s", port_.c_str(), strerror(errno));
    throw std::runtime_error("Could not open serial port " + port_);
  }

  termios tty{};
  tcgetattr(fd_, &tty);
  cfsetispeed(&tty, B38400);
  cfsetospeed(&tty, B38400);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
  tty.c_cflag |= CREAD | CLOCAL;
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~(OPOST | ONLCR);
  tty.c_cc[VTIME] = 0;
  tty.c_cc[VMIN]  = 0;
  tcsetattr(fd_, TCSANOW, &tty);
  tcflush(fd_, TCIOFLUSH);

  RCLCPP_INFO(get_logger(), "Opened %s at 38400 baud", port_.c_str());
}

void AcmNode::closePort()
{
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

void AcmNode::lifterCmdCb(const hardware1_com::msg::LifterCmd::SharedPtr msg)
{
  sendLifterFrame(msg->slave_id, msg->speed, msg->position);
}

void AcmNode::sendLifterFrame(uint8_t slave_id, int8_t speed, float position)
{
  // protokol: FF FE | slave_id | speed | posH | posL | cksum
  int16_t pos_s  = static_cast<int16_t>(position);
  uint8_t pos_h  = static_cast<uint8_t>(pos_s >> 8);
  uint8_t pos_l  = static_cast<uint8_t>(pos_s & 0xFF);
  uint8_t cksum  = static_cast<uint8_t>(speed) ^ pos_h ^ pos_l;

  uint8_t frame[7];
  frame[0] = 0xFF;
  frame[1] = 0xFE;
  frame[2] = slave_id;
  frame[3] = static_cast<uint8_t>(speed);
  frame[4] = pos_h;
  frame[5] = pos_l;
  frame[6] = cksum;

  RCLCPP_INFO(get_logger(),
    "TX slave=%d pos=%.0f | %02X %02X %02X %02X %02X %02X %02X",
    slave_id, position,
    frame[0], frame[1], frame[2], frame[3], frame[4], frame[5], frame[6]);

  ssize_t n = write(fd_, frame, sizeof(frame));
  if (n != static_cast<ssize_t>(sizeof(frame))) {
    RCLCPP_WARN(get_logger(), "short write (%zd/7): %s", n, strerror(errno));
  }
}

}  // namespace hardware1_com
