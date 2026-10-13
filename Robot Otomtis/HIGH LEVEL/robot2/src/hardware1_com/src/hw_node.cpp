#include "hardware1_com/hw_node.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>

namespace hardware1_com
{

HwNode::HwNode()
: Node("hw_node"), fd_(-1)
{
  port_     = declare_parameter<std::string>("port", "/dev/ttyUSB0");
  baudrate_ = declare_parameter<int>("baudrate", 115200);

  openPort();

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      last_vx_ = static_cast<int16_t>(msg->linear.x  *  300.0);
      last_vy_ = static_cast<int16_t>(msg->linear.y  *  300.0);
      last_wz_ = static_cast<int16_t>(msg->angular.z * -300.0);
    });

  lifter_robot1_sub_ = create_subscription<std_msgs::msg::Int16>(
    "lifter_robot1", 10,
    [this](const std_msgs::msg::Int16::SharedPtr msg) {
      last_lifter_robot1_ = std::max<int16_t>(-1050, std::min<int16_t>(0, msg->data));
    });

  lifter_robot2_sub_ = create_subscription<std_msgs::msg::Int16>(
    "lifter_robot2", 10,
    [this](const std_msgs::msg::Int16::SharedPtr msg) {
      last_lifter_robot2_ = std::max<int16_t>(-1050, std::min<int16_t>(0, msg->data));
    });

  sudut_turet_utama_sub_ = create_subscription<std_msgs::msg::Int16>(
    "sudut_turet_utama", 10,
    [this](const std_msgs::msg::Int16::SharedPtr msg) { last_sudut_turet_utama_ = msg->data; });

  sudut_turet1_sub_ = create_subscription<std_msgs::msg::Int16>(
    "sudut_turet1", 10,
    [this](const std_msgs::msg::Int16::SharedPtr msg) { last_sudut_turet1_ = msg->data; });

  sudut_turet2_sub_ = create_subscription<std_msgs::msg::Int16>(
    "sudut_turet2", 10,
    [this](const std_msgs::msg::Int16::SharedPtr msg) { last_sudut_turet2_ = msg->data; });

  lifter_turet1_sub_ = create_subscription<std_msgs::msg::Int16>(
    "lifter_turet1", 10,
    [this](const std_msgs::msg::Int16::SharedPtr msg) { last_lifter_turet1_ = msg->data; });

  lifter_turet2_sub_ = create_subscription<std_msgs::msg::Int16>(
    "lifter_turet2", 10,
    [this](const std_msgs::msg::Int16::SharedPtr msg) { last_lifter_turet2_ = msg->data; });

  griper1_sub_ = create_subscription<std_msgs::msg::Int16>(
    "griper1", 10,
    [this](const std_msgs::msg::Int16::SharedPtr msg) { last_griper1_ = msg->data; });

  griper2_sub_ = create_subscription<std_msgs::msg::Int16>(
    "griper2", 10,
    [this](const std_msgs::msg::Int16::SharedPtr msg) { last_griper2_ = msg->data; });

  griper3_sub_ = create_subscription<std_msgs::msg::Int16>(
    "griper3", 10,
    [this](const std_msgs::msg::Int16::SharedPtr msg) { last_griper3_ = msg->data; });

  griper4_sub_ = create_subscription<std_msgs::msg::Int16>(
    "griper4", 10,
    [this](const std_msgs::msg::Int16::SharedPtr msg) { last_griper4_ = msg->data; });

  sudut_dynamixel1_sub_ = create_subscription<std_msgs::msg::Int16>(
    "sudut_dynamixel1", 10,
    [this](const std_msgs::msg::Int16::SharedPtr msg) { last_sudut_dynamixel1_ = msg->data; });

  penumatic_sub_ = create_subscription<std_msgs::msg::Int16>(
    "penumatic", 10,
    [this](const std_msgs::msg::Int16::SharedPtr msg) { last_penumatic_ = msg->data; });

  lifter_base_belakang_sub_ = create_subscription<std_msgs::msg::Int16>(
    "lifter_base_belakang", 10,
    [this](const std_msgs::msg::Int16::SharedPtr msg) { last_lifter_base_belakang_ = msg->data; });

  timer_ = create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&HwNode::sendFrame, this));
}

HwNode::~HwNode()
{
  closePort();
}

void HwNode::openPort()
{
  fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    RCLCPP_FATAL(get_logger(), "Failed to open %s: %s", port_.c_str(), strerror(errno));
    throw std::runtime_error("Could not open serial port " + port_);
  }

  termios tty{};
  tcgetattr(fd_, &tty);

  speed_t spd = B115200;
  if (baudrate_ == 921600)      spd = B921600;
  else if (baudrate_ == 460800) spd = B460800;
  else if (baudrate_ == 230400) spd = B230400;

  cfsetispeed(&tty, spd);
  cfsetospeed(&tty, spd);
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

  RCLCPP_INFO(get_logger(), "Opened %s at %d baud", port_.c_str(), baudrate_);
}

void HwNode::closePort()
{
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

uint8_t HwNode::checksum(const uint8_t * data, int len)
{
  unsigned int sum = 0;
  for (int i = 0; i < len; ++i) sum += data[i];
  return static_cast<uint8_t>(sum % 256);
}

// TX Frame (38 byte):
// [0]     = 0xAA
// [1]     = 0x55
// [2]     = 0x01
// [3-4]   = vx                (int16 LE)
// [5-6]   = vy                (int16 LE)
// [7-8]   = wz                (int16 LE)
// [9-10]  = lifterRobot2      (int16 LE)
// [11-12] = lifterRobot1      (int16 LE)
// [13-14] = sudutTuretUtama   (int16 LE)
// [15-16] = sudutTuret1       (int16 LE)
// [17-18] = sudutTuret2       (int16 LE)
// [19-20] = lifterTuret1      (int16 LE)
// [21-22] = lifterTuret2      (int16 LE)
// [23-24] = griper1           (int16 LE)
// [25-26] = griper2           (int16 LE)
// [27-28] = griper3           (int16 LE)
// [29-30] = griper4           (int16 LE)
// [31-32] = sudutDynamixel1   (int16 LE)
// [33-34] = sudutDynamixel2   (int16 LE)
// [35-36] = lifterBaseBelakang (int16 LE)
// [37]    = checksum
void HwNode::sendFrame()
{
  uint8_t frame[TX_LEN];
  frame[0] = TX_H0;
  frame[1] = TX_H1;
  frame[2] = TX_TYPE;

  auto put16 = [&](int idx, int16_t val) {
    frame[idx]     = static_cast<uint8_t>(val & 0xFF);
    frame[idx + 1] = static_cast<uint8_t>((val >> 8) & 0xFF);
  };

  put16(3,  last_vx_);
  put16(5,  last_vy_);
  put16(7,  last_wz_);
  put16(9,  last_lifter_robot1_);
  put16(11, last_lifter_robot2_);
  put16(13, last_sudut_turet_utama_);
  put16(15, last_sudut_turet1_);
  put16(17, last_sudut_turet2_);
  put16(19, last_lifter_turet1_);
  put16(21, last_lifter_turet2_);
  put16(23, last_griper1_);
  put16(25, last_griper2_);
  put16(27, last_griper3_);
  put16(29, last_griper4_);
  put16(31, last_sudut_dynamixel1_);
  put16(33, last_penumatic_);
  put16(35, last_lifter_base_belakang_);
  frame[37] = checksum(frame, TX_LEN - 1);


  // put16(3,  last_vx_);
  // put16(5,  last_vy_);
  // put16(7,  last_wz_);
  // put16(9,  last_lifter_robot2_);
  // put16(11, last_lifter_robot1_);
  // put16(13, last_sudut_turet_utama_);
  // put16(15, last_sudut_turet1_);
  // put16(17, 700);
  // put16(19, last_lifter_turet1_);
  // put16(21, -3000);
  // put16(23, last_griper1_);
  // put16(25, last_griper2_);
  // put16(27, 1);
  // put16(29, last_griper4_);
  // put16(31, last_sudut_dynamixel1_);
  // put16(33, last_penumatic_);
  // put16(35, last_lifter_base_belakang_);
  // frame[37] = checksum(frame, TX_LEN - 1);

  ssize_t n = write(fd_, frame, TX_LEN);
  if (n != TX_LEN) {
    RCLCPP_WARN(get_logger(), "sendFrame: short write (%zd/%d)", n, TX_LEN);
  }
}

}  // namespace hardware1_com
