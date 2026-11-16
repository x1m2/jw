cmake_minimum_required(VERSION 3.5)
project(super_lio VERSION 2.0 LANGUAGES CXX C)

set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE STRING "Suppress developer warnings" FORCE)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -pthread -fexceptions")
set(CMAKE_C_FLAGS_RELEASE "-O3 -Wall -fPIC")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--copy-dt-needed-entries")

add_definitions(-DROOT=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions -O0 -g -fsanitize=address")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
include(ProcessorCount)
ProcessorCount(N)
if(N LESS 1)
  set(N 1)
endif()
if(N GREATER 8)
  set(MP_PROC_NUM 4)
  set(BNB_PROC_NUM 6)
elseif(N GREATER 6)
  set(MP_PROC_NUM 3)
  set(BNB_PROC_NUM 4)
elseif(N GREATER 1)
  set(MP_PROC_NUM 2)
  set(BNB_PROC_NUM 2)
else()
  set(MP_PROC_NUM 1)
  set(BNB_PROC_NUM 1)
endif()
add_definitions(-DMP_PROC_NUM=${MP_PROC_NUM})
add_definitions(-DBNB_PROC_NUM=${BNB_PROC_NUM})

find_package(OpenMP REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
message(STATUS "[super_lio] cores=${N} MP_PROC_NUM=${MP_PROC_NUM} BNB_PROC_NUM=${BNB_PROC_NUM} OpenMP_CXX_FLAGS='${OpenMP_CXX_FLAGS}'")

option(ENABLE_OPENCL_BBS "Link the installed OpenCL BBS backend" ON)
set(BBS3D_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../../install/bbs3d"
  CACHE PATH "3d_bbs installation prefix")
set(bbs3d_DIR "${BBS3D_ROOT}/lib/cmake/bbs3d")
find_package(bbs3d CONFIG REQUIRED PATHS "${bbs3d_DIR}" NO_DEFAULT_PATH)
if(ENABLE_OPENCL_BBS AND NOT TARGET bbs3d::opencl_bbs3d)
  message(FATAL_ERROR "Installed 3d_bbs has no OpenCL backend. Run build_opencl.sh, or set ENABLE_OPENCL_BBS=OFF for CPU.")
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(slam_manager REQUIRED)
find_package(slam_manager_interfaces REQUIRED)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(Glog REQUIRED)

find_package(basic REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Glog_INCLUDE_DIRS}
)

set(BACKEND_SOURCES
  include/backend/backend/Backend.cpp
  include/backend/backend/Relocalization.cpp
  include/backend/global_localization/scancontext/Scancontext.cpp
  include/backend/global_localization/InitCoordinate.cpp)

file(GLOB_RECURSE LIO_LIB_SRC "src/lio/*.cpp")
file(GLOB_RECURSE LIO_LIB_HEADERS "include/lio/*.hpp" "include/lio/*.h")

add_library(lio src/ROSWrapper.cpp ${LIO_LIB_SRC} ${LIO_LIB_HEADERS} ${BACKEND_SOURCES})

ament_target_dependencies(lio PUBLIC
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  pcl_ros
  livox_ros_driver2
  tf2_ros
  slam_manager
  slam_manager_interfaces
  basic
)

target_link_libraries(lio PUBLIC
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  Boost::filesystem
  stdc++fs
  glog
  pthread
  OpenMP::OpenMP_CXX
  ${cpp_typesupport_target}
)
if(ENABLE_OPENCL_BBS)
  target_compile_definitions(lio PUBLIC BBS_ENABLE_OPENCL=1)
  target_link_libraries(lio PUBLIC bbs3d::opencl_bbs3d)
else()
  target_link_libraries(lio PUBLIC bbs3d::cpu_bbs3d)
endif()

add_executable(superlio_localization_node src/main.cpp)
target_link_libraries(superlio_localization_node
  lio
  Boost::filesystem
  ${cpp_typesupport_target}
)

install(TARGETS
  lio
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
set_target_properties(superlio_localization_node PROPERTIES
  INSTALL_RPATH "$ORIGIN/../lib;${bbs3d_LIBRARY_DIR}")

install(TARGETS
  superlio_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()
