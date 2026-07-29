cmake_minimum_required(VERSION 3.8)
project(robot_runtime)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(${PROJECT_NAME}_core SHARED
  src/bbs_voxel_generator.cpp
  src/localization_controller.cpp
  src/map_storage.cpp
  src/mapping_controller.cpp
  src/navigation_controller.cpp
  src/navigation_task_executor.cpp
  src/robot_runtime_node.cpp
  src/robot_state_machine.cpp
  src/topology_file_writer.cpp
  src/worker_client.cpp
)

target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_core
  geometry_msgs
  rclcpp
  rclcpp_action
  robot_interfaces
  sensor_msgs
  std_msgs
  std_srvs
)

add_executable(robot_runtime_node src/main.cpp)
target_link_libraries(robot_runtime_node ${PROJECT_NAME}_core)
ament_target_dependencies(robot_runtime_node rclcpp)

install(
  TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS robot_runtime_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_include_directories(include)
ament_export_dependencies(
  geometry_msgs rclcpp rclcpp_action robot_interfaces sensor_msgs std_msgs
  std_srvs)

ament_package()
