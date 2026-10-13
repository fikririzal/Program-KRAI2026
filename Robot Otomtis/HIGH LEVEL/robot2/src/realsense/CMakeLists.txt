cmake_minimum_required(VERSION 3.5)
project(realsense)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python script
install(PROGRAMS
  scripts/realsense_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
