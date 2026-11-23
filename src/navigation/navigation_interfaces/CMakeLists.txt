cmake_minimum_required(VERSION 3.8)
project(navigation_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AvoidanceState.msg"
  "msg/ObstacleStopStatus.msg"
  "msg/PlannerMotionMode.msg"
  "msg/EdgePolicy.msg"
  "msg/PathWithPolicy.msg"
  "msg/Bspline.msg"
  "msg/DataDisp.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs std_msgs
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_export_include_directories(include/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
