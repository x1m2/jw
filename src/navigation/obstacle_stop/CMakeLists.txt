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
find_package(sensor_msgs REQUIRED)
find_package(slam_manager REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(obstacle_stop_node
  src/obstacle_stop_node.cpp
  src/obstacle_stop.cpp
)

target_include_directories(obstacle_stop_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(obstacle_stop_node
  geometry_msgs
  livox_ros_driver2
  rclcpp
  sensor_msgs
  slam_manager
  std_msgs
  visualization_msgs
)
target_link_libraries(obstacle_stop_node Eigen3::Eigen)

install(
  TARGETS obstacle_stop_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_directional_obstacle_policy
    test/test_directional_obstacle_policy.cpp)
  target_include_directories(test_directional_obstacle_policy PRIVATE include)
endif()

ament_package()
