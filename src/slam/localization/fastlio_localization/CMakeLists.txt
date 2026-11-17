cmake_minimum_required(VERSION 3.5)
project(fastlio_localization)

# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++17 -O3")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions -Wno-unused-local-typedefs -Wno-format-truncation -Wno-unused-result -Wno-pedantic -Wno-unused-parameter -Wno-unused-variable")

# echo "/home/will/coredump/core-%e-%p-%s-%t" > /proc/sys/kernel/core_pattern
# SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)|(aarch32)|(AARCH32)|(aarch64)|(AARCH64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  # if(N GREATER 6)
  #   add_definitions(-DMP_EN)
  #   add_definitions(-DMP_PROC_NUM=4)
  #   add_definitions(-DBNB_PROC_NUM=6)
  #   message("core for MP: 4")
  # else
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
    add_definitions(-DBNB_PROC_NUM=4)
    message("core for MP: 4")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    add_definitions(-DBNB_PROC_NUM=3)
    message("core for MP: 3")
  else()
    add_definitions(-DMP_PROC_NUM=1)
    add_definitions(-DBNB_PROC_NUM=2)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
  add_definitions(-DBNB_PROC_NUM=2)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

option(ENABLE_OPENCL_BBS "Link the installed OpenCL BBS backend" ON)
# Both backends come from one 3d_bbs installation; never build a second copy.
set(BBS3D_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../../install/bbs3d"
  CACHE PATH "3d_bbs installation prefix")
set(bbs3d_DIR "${BBS3D_ROOT}/lib/cmake/bbs3d")
find_package(bbs3d CONFIG REQUIRED PATHS "${bbs3d_DIR}" NO_DEFAULT_PATH)
if(ENABLE_OPENCL_BBS AND NOT TARGET bbs3d::opencl_bbs3d)
  message(FATAL_ERROR "Installed 3d_bbs has no OpenCL backend. Run build_opencl.sh, or set ENABLE_OPENCL_BBS=OFF for CPU.")
endif()

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(slam_manager REQUIRED)
find_package(robot_geometry REQUIRED)
find_package(slam_manager_interfaces REQUIRED)

include_directories(
  ${pcl_conversions_INCLUDE_DIRS}
  include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

add_executable(fastlio_localization_node
  src/main.cpp
  src/ros2_main.cpp
  include/ikd-Tree/ikd_Tree.cpp
  include/global_localization/scancontext/Scancontext.cpp
  include/global_localization/InitCoordinate.cpp
  include/system/frontend/ImuProcessor.cpp
  include/system/frontend/use-ikfom.cpp
  include/system/System.hpp)
target_link_libraries(fastlio_localization_node stdc++fs ${PCL_LIBRARIES})
if(ENABLE_OPENCL_BBS)
  target_compile_definitions(fastlio_localization_node PRIVATE BBS_ENABLE_OPENCL=1)
  target_link_libraries(fastlio_localization_node bbs3d::opencl_bbs3d)
else()
  target_link_libraries(fastlio_localization_node bbs3d::cpu_bbs3d)
endif()
ament_target_dependencies(fastlio_localization_node
  rclcpp sensor_msgs nav_msgs tf2_ros livox_ros_driver2 slam_manager_interfaces
  slam_manager robot_geometry)
set_target_properties(fastlio_localization_node PROPERTIES
  INSTALL_RPATH "$ORIGIN/../lib;${bbs3d_LIBRARY_DIR}")

install(TARGETS fastlio_localization_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config rviz_cfg
  DESTINATION share/${PROJECT_NAME})
