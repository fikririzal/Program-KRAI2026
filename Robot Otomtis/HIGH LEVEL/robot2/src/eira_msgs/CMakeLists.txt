cmake_minimum_required(VERSION 3.8)
project(eira_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  # "msg/MatchConfig.msg"
  # "msg/GridInfo.msg"
  # "msg/PathRequest.msg"
  # "msg/PathResponse.msg"
  # "msg/ObjectPlacement.msg"
  # "msg/ActionCmd.msg"
  # "msg/SensorData.msg"
  "msg/ForestPathPoint.msg"
  "msg/ForestPlanStep.msg"
  "msg/KfsRetrievalPlan.msg"
  "msg/GuiData.msg"
  "msg/WebGuiData.msg"
)

set(srv_files
  # "srv/UpdateKFSPlacement.srv"
  # "srv/PathPlanning.srv"
  # "srv/SetMatchState.srv"
  # "srv/GetMatchState.srv"
  # "srv/SetOdometry.srv"
  "srv/ScanVision.srv"
)

set(action_files
  "action/Count.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
