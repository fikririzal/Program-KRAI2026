#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int16.hpp"

/* ================================================================== */
/*  ISIKAN NILAI DUMMY DI SINI                                        */
/* ================================================================== */

// cmd_vel → vx vy wz (dalam m/s dan rad/s, di-scale ×300 di hw_node)
static constexpr double DUMMY_VX =  1.0;   // m/s
static constexpr double DUMMY_VY =  2.0;   // m/s
static constexpr double DUMMY_WZ =  3.0;   // rad/s

// lifter robot (satuan bebas, dikirim as-is int16)
static constexpr int16_t DUMMY_LIFTER_ROBOT1 = -1004;
static constexpr int16_t DUMMY_LIFTER_ROBOT2 = -500;

// sudut turet (satuan bebas, dikirim as-is int16)
static constexpr int16_t DUMMY_SUDUT_TURET_UTAMA = 11;
static constexpr int16_t DUMMY_SUDUT_TURET1      = 12;
static constexpr int16_t DUMMY_SUDUT_TURET2      = 32;

// lifter turet
static constexpr int16_t DUMMY_LIFTER_TURET1 = 45;
static constexpr int16_t DUMMY_LIFTER_TURET2 = 50;

// griper
static constexpr int16_t DUMMY_GRIPER1 = 42;
static constexpr int16_t DUMMY_GRIPER2 = 78;
static constexpr int16_t DUMMY_GRIPER3 = 122;
static constexpr int16_t DUMMY_GRIPER4 = 90;

// sudut dynamixel
static constexpr int16_t DUMMY_SUDUT_DYNAMIXEL1 = 30;
static constexpr int16_t DUMMY_SUDUT_DYNAMIXEL2 = 170;

/* ================================================================== */

class DummyPub : public rclcpp::Node
{
public:
  DummyPub() : Node("test_dummy_pub")
  {
    pub_cmd_vel_          = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    pub_lifter_robot1_    = create_publisher<std_msgs::msg::Int16>("lifter_robot1", 10);
    pub_lifter_robot2_    = create_publisher<std_msgs::msg::Int16>("lifter_robot2", 10);
    pub_sudut_turet_utama_= create_publisher<std_msgs::msg::Int16>("sudut_turet_utama", 10);
    pub_sudut_turet1_     = create_publisher<std_msgs::msg::Int16>("sudut_turet1", 10);
    pub_sudut_turet2_     = create_publisher<std_msgs::msg::Int16>("sudut_turet2", 10);
    pub_lifter_turet1_    = create_publisher<std_msgs::msg::Int16>("lifter_turet1", 10);
    pub_lifter_turet2_    = create_publisher<std_msgs::msg::Int16>("lifter_turet2", 10);
    pub_griper1_          = create_publisher<std_msgs::msg::Int16>("griper1", 10);
    pub_griper2_          = create_publisher<std_msgs::msg::Int16>("griper2", 10);
    pub_griper3_          = create_publisher<std_msgs::msg::Int16>("griper3", 10);
    pub_griper4_          = create_publisher<std_msgs::msg::Int16>("griper4", 10);
    pub_sudut_dynamixel1_ = create_publisher<std_msgs::msg::Int16>("sudut_dynamixel1", 10);
    pub_sudut_dynamixel2_ = create_publisher<std_msgs::msg::Int16>("sudut_dynamixel2", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&DummyPub::publish_all, this));

    RCLCPP_INFO(get_logger(), "test_dummy_pub running — edit konstanta di test_dummy_pub.cpp lalu rebuild");
  }

private:
  void publish_all()
  {
    // cmd_vel
    geometry_msgs::msg::Twist twist;
    twist.linear.x  = DUMMY_VX;
    twist.linear.y  = DUMMY_VY;
    twist.angular.z = DUMMY_WZ;
    pub_cmd_vel_->publish(twist);

    // helper lambda
    auto pub16 = [](auto & pub, int16_t val) {
      std_msgs::msg::Int16 msg;
      msg.data = val;
      pub->publish(msg);
    };

    pub16(pub_lifter_robot1_,     DUMMY_LIFTER_ROBOT1);
    pub16(pub_lifter_robot2_,     DUMMY_LIFTER_ROBOT2);
    pub16(pub_sudut_turet_utama_, DUMMY_SUDUT_TURET_UTAMA);
    pub16(pub_sudut_turet1_,      DUMMY_SUDUT_TURET1);
    pub16(pub_sudut_turet2_,      DUMMY_SUDUT_TURET2);
    pub16(pub_lifter_turet1_,     DUMMY_LIFTER_TURET1);
    pub16(pub_lifter_turet2_,     DUMMY_LIFTER_TURET2);
    pub16(pub_griper1_,           DUMMY_GRIPER1);
    pub16(pub_griper2_,           DUMMY_GRIPER2);
    pub16(pub_griper3_,           DUMMY_GRIPER3);
    pub16(pub_griper4_,           DUMMY_GRIPER4);
    pub16(pub_sudut_dynamixel1_,  DUMMY_SUDUT_DYNAMIXEL1);
    pub16(pub_sudut_dynamixel2_,  DUMMY_SUDUT_DYNAMIXEL2);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_lifter_robot1_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_lifter_robot2_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_sudut_turet_utama_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_sudut_turet1_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_sudut_turet2_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_lifter_turet1_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_lifter_turet2_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_griper1_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_griper2_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_griper3_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_griper4_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_sudut_dynamixel1_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_sudut_dynamixel2_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyPub>());
  rclcpp::shutdown();
  return 0;
}
