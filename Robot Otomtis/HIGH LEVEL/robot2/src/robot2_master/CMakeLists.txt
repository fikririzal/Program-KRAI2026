cmake_minimum_required(VERSION 3.5)
project(robot2_master)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware1_com REQUIRED)
find_package(sensor_tof REQUIRED)
find_package(eira_msgs REQUIRED)

include_directories(include)



add_executable(seq_node src/seq_node.cpp)
ament_target_dependencies(seq_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  hardware1_com
  sensor_tof
  eira_msgs
)

install(TARGETS seq_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
