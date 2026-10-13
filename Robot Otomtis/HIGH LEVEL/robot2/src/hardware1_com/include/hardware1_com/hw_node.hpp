#ifndef HARDWARE1_COM__HW_NODE_HPP_
#define HARDWARE1_COM__HW_NODE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"

namespace hardware1_com
{

class HwNode : public rclcpp::Node
{
public:
  HwNode();
  ~HwNode() override;

private:
  // TX: AA 55 01 | vx vy wz lifterR2 lifterR1 sudutUtama sudutT1 sudutT2
  //               lifterT1 lifterT2 grip1 grip2 grip3 grip4 dynx1 dynx2 | chk
  // 17 field × int16 LE + 3 header + 1 chk = 38 bytes
  static constexpr uint8_t TX_H0   = 0xAA;
  static constexpr uint8_t TX_H1   = 0x55;
  static constexpr uint8_t TX_TYPE = 0x01;
  static constexpr int     TX_LEN  = 38;

  void openPort();
  void closePort();
  void sendFrame();

  static uint8_t checksum(const uint8_t * data, int len);

  std::string port_;
  int baudrate_;
  int fd_;

  // TX fields
  int16_t last_vx_{0};
  int16_t last_vy_{0};
  int16_t last_wz_{0};
  int16_t last_lifter_robot1_{0};
  int16_t last_lifter_robot2_{0};
  int16_t last_sudut_turet_utama_{0};
  int16_t last_sudut_turet1_{0};
  int16_t last_sudut_turet2_{0};
  int16_t last_lifter_turet1_{0};
  int16_t last_lifter_turet2_{0};
  int16_t last_griper1_{0};
  int16_t last_griper2_{0};
  int16_t last_griper3_{0};
  int16_t last_griper4_{0};
  int16_t last_sudut_dynamixel1_{0};
  int16_t last_penumatic_{0};
  int16_t last_lifter_base_belakang_{0};

  // Subscriptions (TX data sources)
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr lifter_robot1_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr lifter_robot2_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sudut_turet_utama_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sudut_turet1_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sudut_turet2_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr lifter_turet1_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr lifter_turet2_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr griper1_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr griper2_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr griper3_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr griper4_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sudut_dynamixel1_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr penumatic_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr lifter_base_belakang_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace hardware1_com

#endif  // HARDWARE1_COM__HW_NODE_HPP_
