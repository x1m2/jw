cmake_minimum_required(VERSION 3.8)
project(topology_map_generator)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 默认 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

##################################################
# find package
##################################################

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

##################################################
# include
##################################################

include_directories(
  include
)

##################################################
# executable
##################################################

add_executable(topology_map_generator_node
  src/topology_map_generator_node.cpp
  src/main.cpp
)

##################################################
# dependencies
##################################################

ament_target_dependencies(
  topology_map_generator_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
)

##################################################
# install
##################################################

install(
  TARGETS topology_map_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

##################################################
# export
##################################################
ament_package()