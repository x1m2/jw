cmake_minimum_required(VERSION 3.8)
project(woodox_imu_bridge)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(woodox_ros2_interface REQUIRED)

add_executable(woodox_imu_bridge_node src/woodox_imu_bridge_node.cpp)
target_compile_features(woodox_imu_bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(
  woodox_imu_bridge_node
  rcl
  rclcpp
  sensor_msgs
  woodox_ros2_interface
)

install(TARGETS woodox_imu_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
