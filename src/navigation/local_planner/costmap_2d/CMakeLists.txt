cmake_minimum_required(VERSION 3.8)
project(costmap_2d)

# 编译标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# 包含目录
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# PCL 定义
add_definitions(${PCL_DEFINITIONS})

# PCL 链接目录
link_directories(${PCL_LIBRARY_DIRS})

# 创建库
add_library(${PROJECT_NAME}
  src/costmap_2d.cpp
)

# ament 依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  livox_ros_driver2
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
)

# 链接 PCL
target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

# 创建节点
add_executable(costmap_2d_node
  src/costmap_2d_node.cpp
)

# 节点依赖
ament_target_dependencies(costmap_2d_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  livox_ros_driver2
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
)

# 链接库
target_link_libraries(costmap_2d_node
  ${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

# 安装库
install(
  TARGETS ${PROJECT_NAME} costmap_2d_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

# 安装 launch 文件
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装 config 文件
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 导出 include
ament_export_include_directories(
  include
)

# 导出库
ament_export_libraries(
  ${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  livox_ros_driver2
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  PCL
  Eigen3
)

ament_package()
