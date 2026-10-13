cmake_minimum_required(VERSION 3.8)
project(kfs_plan_executor_dummy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eira_msgs REQUIRED)

add_executable(kfs_plan_executor_dummy_node
  src/kfs_plan_executor_dummy_node.cpp
)

ament_target_dependencies(kfs_plan_executor_dummy_node
  rclcpp
  eira_msgs
)

install(TARGETS kfs_plan_executor_dummy_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
