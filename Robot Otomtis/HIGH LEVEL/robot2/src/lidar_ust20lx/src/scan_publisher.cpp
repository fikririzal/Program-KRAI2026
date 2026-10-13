#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanPublisher : public rclcpp::Node
{
public:
  ScanPublisher()
  : Node("scan_publisher")
  {
    this->declare_parameter<std::string>("input_topic", "/scan");
    this->declare_parameter<std::string>("output_topic", "/scan_filtered");
    this->declare_parameter<double>("range_min", 0.06);
    this->declare_parameter<double>("range_max", 20.0);
    this->declare_parameter<double>("angle_min", -M_PI);
    this->declare_parameter<double>("angle_max", M_PI);

    input_topic_  = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    range_min_    = this->get_parameter("range_min").as_double();
    range_max_    = this->get_parameter("range_max").as_double();
    angle_min_    = this->get_parameter("angle_min").as_double();
    angle_max_    = this->get_parameter("angle_max").as_double();

    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ScanPublisher::scan_callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      output_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(),
      "UST-20LX scan_publisher aktif: %s -> %s | range [%.2f, %.2f] m | angle [%.2f, %.2f] rad",
      input_topic_.c_str(), output_topic_.c_str(),
      range_min_, range_max_, angle_min_, angle_max_);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    auto out = std::make_unique<sensor_msgs::msg::LaserScan>(*msg);

    float angle = msg->angle_min;
    for (size_t i = 0; i < out->ranges.size(); ++i, angle += msg->angle_increment) {
      if (angle < angle_min_ || angle > angle_max_) {
        out->ranges[i] = std::numeric_limits<float>::infinity();
        continue;
      }
      if (out->ranges[i] < range_min_ || out->ranges[i] > range_max_) {
        out->ranges[i] = std::numeric_limits<float>::infinity();
      }
    }

    pub_->publish(std::move(out));
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;

  std::string input_topic_;
  std::string output_topic_;
  double range_min_;
  double range_max_;
  double angle_min_;
  double angle_max_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanPublisher>());
  rclcpp::shutdown();
  return 0;
}
