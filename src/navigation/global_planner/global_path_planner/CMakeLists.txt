cmake_minimum_required(VERSION 3.8)
project(global_path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common search io)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib
  src/global_path_planner.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

target_link_libraries(${PROJECT_NAME}_lib
  ${PCL_LIBRARIES}
)

add_executable(global_path_planner_node
  src/global_path_planner_node.cpp
)

ament_target_dependencies(global_path_planner_node
  rclcpp
)

target_link_libraries(global_path_planner_node
  ${PROJECT_NAME}_lib
)

install(TARGETS
  ${PROJECT_NAME}_lib
  global_path_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(PROGRAMS
  scripts/simulate_odom.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
