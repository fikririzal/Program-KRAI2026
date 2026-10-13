#include "rclcpp/rclcpp.hpp"
#include "hardware1_com/acm_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hardware1_com::AcmNode>());
  rclcpp::shutdown();
  return 0;
}
