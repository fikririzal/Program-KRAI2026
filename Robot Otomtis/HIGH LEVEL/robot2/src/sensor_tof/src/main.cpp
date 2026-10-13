#include "rclcpp/rclcpp.hpp"
#include "sensor_tof/tof_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sensor_tof::TofNode>());
  rclcpp::shutdown();
  return 0;
}
