#ifndef HARDWARE1_COM__ACM_NODE_HPP_
#define HARDWARE1_COM__ACM_NODE_HPP_

#include <cstdint>
#include <string>

#include "hardware1_com/msg/lifter_cmd.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hardware1_com
{

// Protocol (7 bytes):
// FF FE | slave_id | speed(int8) | posH | posL | cksum
// cksum = speed ^ posH ^ posL
class AcmNode : public rclcpp::Node
{
public:
  AcmNode();
  ~AcmNode() override;

private:
  void openPort();
  void closePort();
  void lifterCmdCb(const hardware1_com::msg::LifterCmd::SharedPtr msg);
  void sendLifterFrame(uint8_t slave_id, int8_t speed, float position);

  std::string port_;
  int fd_;

  rclcpp::Subscription<hardware1_com::msg::LifterCmd>::SharedPtr lifter_cmd_sub_;
};

}  // namespace hardware1_com

#endif  // HARDWARE1_COM__ACM_NODE_HPP_
