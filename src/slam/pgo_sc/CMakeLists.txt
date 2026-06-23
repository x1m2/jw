cmake_minimum_required(VERSION 3.10)
project(pgo_sc)

## 1. 编译器设置 (保持原有的高性能编译选项)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-pthread -fexceptions -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra ${CMAKE_CXX_FLAGS}")

## 2. 查找依赖项
# 查找 OpenMP (用于 Nano-GICP 和 Quatro)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# 查找 ROS 2 核心包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(ament_index_cpp REQUIRED)
# rosbag2
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
# 查找第三方非 ROS 库
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)

# 假设 nano_gicp 和 quatro 也是作为 ament 包安装的
find_package(nano_gicp REQUIRED)
find_package(quatro REQUIRED)

## 3. 头文件目录
include_directories(
  include
  src/scancontext_tro
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${TEASER_INCLUDE_DIRS}
)

## 4. 构建目标
# 定义公共依赖库
set(TEASER_LIBRARIES
  /usr/local/lib/libteaser_registration.so
  /usr/local/lib/libteaser_io.so
)
set(COMMON_LIBS
  ${PCL_LIBRARIES}
  ${GTSAM_LIBRARIES}
  OpenMP::OpenMP_CXX
  gtsam
  ${TEASER_LIBRARIES} 
)

set(ROS_DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  message_filters
  ament_index_cpp
  nano_gicp
  quatro
  rosbag2_cpp
  rosbag2_storage
)

# A. 构建核心功能库 (Modules)
add_library(${PROJECT_NAME}_modules SHARED
  src/scancontext_tro/Scancontext.cpp
  src/loop_closure.cpp
  src/pgo_sc.cpp
)
# 在 ROS 2 中推荐使用 ament_target_dependencies 处理依赖，它会自动处理 include 和 link
ament_target_dependencies(${PROJECT_NAME}_modules ${ROS_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_modules ${COMMON_LIBS})

# B. 构建可执行文件 (Node)
add_executable(${PROJECT_NAME}_node src/main.cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${ROS_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_node 
  ${PROJECT_NAME}_modules
  ${COMMON_LIBS}
)

## 5. 安装逻辑 (ROS 2 必须)
install(TARGETS
  ${PROJECT_NAME}_modules
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

## 6. Ament 导出
ament_export_libraries(${PROJECT_NAME}_modules)
ament_export_dependencies(${ROS_DEPENDENCIES} PCL GTSAM Eigen3)
ament_package()