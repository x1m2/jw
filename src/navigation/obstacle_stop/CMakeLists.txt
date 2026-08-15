cmake_minimum_required(VERSION 3.8)
project(obstacle_stop)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(slam_manager REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/obstacle_stop.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  livox_ros_driver2
  rclcpp
  std_msgs
  visualization_msgs
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

add_executable(obstacle_stop_node src/obstacle_stop_node.cpp)
ament_target_dependencies(obstacle_stop_node rclcpp slam_manager)
target_link_libraries(obstacle_stop_node ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} obstacle_stop_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(
  Eigen3 geometry_msgs livox_ros_driver2 rclcpp std_msgs
  visualization_msgs)

ament_package()
