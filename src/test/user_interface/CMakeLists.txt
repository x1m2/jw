cmake_minimum_required(VERSION 3.8)
project(user_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_runtime REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/navigation_task_state_machine.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
)

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

install(
  TARGETS ${PROJECT_NAME} navigation_task_state_machine_node
  EXPORT export_${PROJECT_NAME}
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

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs rclcpp std_msgs)

ament_package()
