#include "rclcpp/rclcpp.hpp"
#include "hardware1_com/hw_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hardware1_com::HwNode>());
  rclcpp::shutdown();
  return 0;
}
