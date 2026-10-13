#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <sensor_tof/msg/tof_array.hpp>
#include <cstdio>

#define RESET   "\033[0m"
#define BOLD    "\033[1m"
#define CYAN    "\033[36m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define MAGENTA "\033[35m"
#define BLUE    "\033[34m"
#define RED     "\033[31m"
#define CLEAR   "\033[2J\033[H"

class MonitorNode : public rclcpp::Node
{
public:
  MonitorNode() : Node("monitor_node")
  {
    tof_sub_ = create_subscription<sensor_tof::msg::TofArray>(
      "tof/array", 10,
      [this](sensor_tof::msg::TofArray::SharedPtr m) {
        tof_dk_    = m->sensor1_depan;
        tof_dkn_   = m->sensor2_depan;
        tof_sk_    = m->sensor1_kiri;
        tof_skn_   = m->sensor2_kanan;
        tof_mid_[0] = m->sensor_mid1;
        tof_mid_[1] = m->sensor_mid2;
        tof_mid_[2] = m->sensor_mid3;
        tof_mid_[3] = m->sensor_mid4; });

    cmdvel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr m) {
        vx_ = m->linear.x; vy_ = m->linear.y; wz_ = m->angular.z; });

    imu_yaw_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/imu_yaw", 10,
      [this](std_msgs::msg::Float32::SharedPtr m) { imu_yaw_ = m->data; });

    wall_kanan_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/wall/kanan", 10,
      [this](std_msgs::msg::Float32::SharedPtr m) { wall_kanan_ = m->data; });

    wall_kiri_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/wall/kiri", 10,
      [this](std_msgs::msg::Float32::SharedPtr m) { wall_kiri_ = m->data; });

    pos_debug_sub_ = create_subscription<std_msgs::msg::String>(
      "/pos_debug", 10,
      [this](std_msgs::msg::String::SharedPtr m) { pos_debug_ = m->data; });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&MonitorNode::print, this));
  }

private:
  void print()
  {
    printf(CLEAR);
    printf(BOLD CYAN "╔══════════════════════════════════════════╗\n" RESET);
    printf(BOLD CYAN "║           ROBOT2  MONITOR                ║\n" RESET);
    printf(BOLD CYAN "╚══════════════════════════════════════════╝\n\n" RESET);

    printf(BOLD YELLOW "  [ TOF SENSORS (mm) ]\n" RESET);
    printf("    depan_kiri  (s1): %5u    depan_kanan (s2): %5u\n", tof_dk_, tof_dkn_);
    printf("    samping_kiri(s3): %5u    samping_kanan(s4): %5u\n", tof_sk_, tof_skn_);
    printf("    mid1: %5u   mid2: %5u   mid3: %5u   mid4: %5u\n\n",
           tof_mid_[0], tof_mid_[1], tof_mid_[2], tof_mid_[3]);

    printf(BOLD MAGENTA "  [ IMU YAW ]\n" RESET);
    printf("    yaw : %7.2f deg\n\n", imu_yaw_);

    printf(BOLD YELLOW "  [ LIDAR WALLS (mm) ]\n" RESET);
    printf("    kanan : %7.1f mm\n", wall_kanan_);
    printf("    kiri  : %7.1f mm\n\n", wall_kiri_);

    printf(BOLD GREEN "  [ CMD VEL ]\n" RESET);
    printf("    Vx : %7.3f    Vy : %7.3f    Wz : %7.3f\n\n", vx_, vy_, wz_);

    printf(BOLD CYAN "  [ AUTO ERROR ]\n" RESET);
    if (pos_debug_.empty()) {
      printf(YELLOW "    (menunggu fase auto...)\n" RESET);
    } else {
      printf(GREEN "    %s\n" RESET, pos_debug_.c_str());
    }
    printf("\n");
    fflush(stdout);
  }

  uint32_t tof_dk_{0}, tof_dkn_{0}, tof_sk_{0}, tof_skn_{0};
  uint32_t tof_mid_[4]{};
  double   vx_{0}, vy_{0}, wz_{0};
  float    imu_yaw_{0.f};
  float    wall_kanan_{-1.f};
  float    wall_kiri_{-1.f};
  std::string pos_debug_;

  rclcpp::Subscription<sensor_tof::msg::TofArray>::SharedPtr        tof_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr        cmdvel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr           imu_yaw_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr           wall_kanan_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr           wall_kiri_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr            pos_debug_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MonitorNode>());
  rclcpp::shutdown();
  return 0;
}
