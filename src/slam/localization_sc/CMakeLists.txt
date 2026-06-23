cmake_minimum_required(VERSION 3.10)
project(localization_sc LANGUAGES CXX)

# -----------------------------
# Compiler settings
# -----------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions -g -ggdb -O3 -Wall -Wextra")
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)

# OpenMP
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
  message(FATAL_ERROR "OpenMP could not be found.")
endif()

# -----------------------------
# ROS 2 and other dependencies
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(nano_gicp REQUIRED)
find_package(quatro REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(angles REQUIRED)
# -----------------------------
# Include directories
# -----------------------------
include_directories(
  include
  src/scancontext
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${TEASER_INCLUDE_DIRS}
)
set(TEASER_LIBRARIES
  /usr/local/lib/libteaser_registration.so
  /usr/local/lib/libteaser_io.so
)
# -----------------------------
# All common dependencies
# -----------------------------
set(COMMON_DEPENDENCIES
  rclcpp
  angles
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_eigen
  pcl_conversions
  pcl_ros
  nano_gicp
  quatro
  rosbag2_cpp
  rosbag2_storage
)

# -----------------------------
# Build library
# -----------------------------
add_library(${PROJECT_NAME}_modules
  src/scancontext/Scancontext.cpp
  src/localization_sc.cpp
  src/map_matcher.cpp
)

ament_target_dependencies(${PROJECT_NAME}_modules ${COMMON_DEPENDENCIES})

target_link_libraries(${PROJECT_NAME}_modules
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBS}
  ${OpenMP_LIBS}
  ${TEASER_LIBRARIES} 
)

# -----------------------------
# Build executable
# -----------------------------
add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_modules)
ament_target_dependencies(${PROJECT_NAME}_node ${COMMON_DEPENDENCIES})

# -----------------------------
# Install targets
# -----------------------------
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