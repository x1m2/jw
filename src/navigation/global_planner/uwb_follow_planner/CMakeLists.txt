cmake_minimum_required(VERSION 3.8)
project(uwb_follow_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(navigation_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(slam_manager REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(uwb_aoa_pkg REQUIRED)

add_library(${PROJECT_NAME}_lib
  src/uwb_follow_planner.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_lib
  geometry_msgs
  nav_msgs
  navigation_interfaces
  rclcpp
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  uwb_aoa_pkg
)

add_executable(uwb_follow_planner_node
  src/uwb_follow_planner_node.cpp
)
ament_target_dependencies(uwb_follow_planner_node rclcpp slam_manager)
target_link_libraries(uwb_follow_planner_node ${PROJECT_NAME}_lib)

install(TARGETS
  ${PROJECT_NAME}_lib
  uwb_follow_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()
