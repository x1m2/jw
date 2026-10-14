cmake_minimum_required(VERSION 3.8)
project(slam_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(navigation_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(slam_manager_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(woodox_ros2_interface REQUIRED)

add_library(${PROJECT_NAME}_core STATIC
  src/localization.cpp
  src/map_check.cpp
  src/mapping.cpp
  src/navigation.cpp
  src/navigation_task.cpp
  src/slam_manager_node.cpp
  src/worker_runtime.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core
  geometry_msgs
  navigation_interfaces
  rclcpp
  rclcpp_action
  slam_manager_interfaces
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  woodox_ros2_interface
)
target_link_libraries(${PROJECT_NAME}_core
  Eigen3::Eigen
  nlohmann_json::nlohmann_json
)

add_executable(slam_manager_node src/main.cpp)
target_link_libraries(slam_manager_node ${PROJECT_NAME}_core)
ament_target_dependencies(slam_manager_node rclcpp)

install(
  TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS slam_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_include_directories(include)
ament_export_dependencies(
  geometry_msgs navigation_interfaces rclcpp rclcpp_action
  slam_manager_interfaces sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs
  woodox_ros2_interface nlohmann_json)

ament_package()
