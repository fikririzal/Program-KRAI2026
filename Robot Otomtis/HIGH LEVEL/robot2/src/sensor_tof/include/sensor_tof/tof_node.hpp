#ifndef SENSOR_TOF__TOF_NODE_HPP_
#define SENSOR_TOF__TOF_NODE_HPP_

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_tof/msg/tof_array.hpp"

namespace sensor_tof
{

struct TofBus
{
  std::string port;
  int fd = -1;
  std::vector<uint8_t> buffer;
};

class TofNode : public rclcpp::Node
{
public:
  TofNode();
  ~TofNode() override;

private:
  static constexpr int kFrameLength = 16;
  static constexpr std::array<uint8_t, 3> kHeader{87, 0, 255};
  static const std::array<std::string, 9> kSensorNames;

  int openPort(const std::string & port);
  void closePorts();
  void poll();
  void pollBus(TofBus & bus);
  static bool verifyCheckSum(const uint8_t * data, int len);

  int baudrate_;
  std::vector<TofBus> buses_;

  std::map<uint8_t, rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr> range_pubs_;
  rclcpp::Publisher<sensor_tof::msg::TofArray>::SharedPtr tof_array_pub_;
  std::array<uint32_t, 9> distances_{};
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sensor_tof

#endif  // SENSOR_TOF__TOF_NODE_HPP_
