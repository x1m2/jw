cmake_minimum_required(VERSION 3.8)
project(slam_manager_interfaces)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MapKeyframe.msg"
  "msg/LocalizationPose.msg"
  "msg/NavigationNode.msg"
  "msg/TaskTarget.msg"
  "msg/RobotPose.msg"
  "msg/RuntimeState.msg"
  "msg/WorkerStatus.msg"
  "action/NavigateToPose.action"
  "action/StartLocalization.action"
  "action/StartNavigation.action"
  "srv/DeleteMap.srv"
  "srv/LoadMap.srv"
  "srv/SetWorkerState.srv"
  "srv/StartMapping.srv"
  DEPENDENCIES action_msgs builtin_interfaces geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
