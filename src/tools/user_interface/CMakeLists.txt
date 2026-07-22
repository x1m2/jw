cmake_minimum_required(VERSION 3.8)

project(user_interface)

# =========================================================
# C++17
# =========================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# =========================================================
# Compile Options
# =========================================================
add_compile_options(
  -Wall
  -Wextra
  -Wpedantic
)

# =========================================================
# Find Packages
# =========================================================
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(robot_runtime REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(cv_bridge REQUIRED)

find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

find_package(ament_index_cpp REQUIRED)


# =========================================================
# Include Directories
# =========================================================
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# =========================================================
# Library
# =========================================================
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/navigation_task_state_machine.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  ament_index_cpp
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
)

# =========================================================
# navigation_task_state_machine_node
# =========================================================
add_executable(navigation_task_state_machine_node
  src/navigation_task_state_machine_node.cpp
)

target_link_libraries(navigation_task_state_machine_node
  ${PROJECT_NAME}
)

ament_target_dependencies(navigation_task_state_machine_node
  rclcpp
  robot_runtime
)

# =========================================================
# Install
# =========================================================

install(
  TARGETS
    ${PROJECT_NAME}
    navigation_task_state_machine_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# =========================================================
# Export
# =========================================================
ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
)

ament_package()
