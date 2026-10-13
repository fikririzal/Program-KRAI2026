cmake_minimum_required(VERSION 3.5)
project(hardware1_com)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── generate message ──────────────────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LifterCmd.msg"
)

include_directories(include)

# ── hw_node: STM32 via USB ────────────────────────────────────────────────────
add_executable(hw_node src/hw_node.cpp src/main.cpp)
ament_target_dependencies(hw_node rclcpp geometry_msgs std_msgs)

# ── test_dummy_pub: kirim nilai dummy ke semua topic hw_node ────────────────
add_executable(test_dummy_pub src/test_dummy_pub.cpp)
ament_target_dependencies(test_dummy_pub rclcpp geometry_msgs std_msgs)

# ── acm_node: slave roda via ACM0 ────────────────────────────────────────────
add_executable(acm_node src/acm_node.cpp src/acm_main.cpp)
ament_target_dependencies(acm_node rclcpp geometry_msgs)
rosidl_target_interfaces(acm_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS hw_node acm_node test_dummy_pub
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
