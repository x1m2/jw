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

set(BUILD_CUDA ON)
if (BUILD_CUDA)
  add_definitions(-DUSE_CUDA)
  find_package(CUDA REQUIRED)
  include_directories(${CUDA_INCLUDE_DIRS})
  link_directories(${CUDA_LIBRARY_DIRS})

  set(CUDA_PROPAGATE_HOST_FLAGS ON)
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -std=c++20 -Xcompiler -fPIC)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(GTSAM REQUIRED)
find_package(Glog REQUIRED)

find_package(basic REQUIRED)

add_library(cpu_bbs3d
  include/backend/global_localization/bbs3d/cpu_bbs3d/bbs3d.cpp
  include/backend/global_localization/bbs3d/cpu_bbs3d/voxelmaps.cpp
  include/backend/global_localization/bbs3d/cpu_bbs3d/voxelmaps_io.cpp
)

if(BUILD_CUDA)
  cuda_add_library(gpu_bbs3d SHARED
    include/backend/global_localization/bbs3d/gpu_bbs3d/bbs3d.cu
    include/backend/global_localization/bbs3d/gpu_bbs3d/calc_score.cu
    include/backend/global_localization/bbs3d/gpu_bbs3d/voxelmaps.cu
    include/backend/global_localization/bbs3d/gpu_bbs3d/stream_manager/check_error.cu
    include/backend/global_localization/bbs3d/gpu_bbs3d/voxelmaps_io.cu
  )
endif()

include_directories(
  include
  3rdparty
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${Glog_INCLUDE_DIRS}
)

set(BACKEND_SOURCES
  include/backend/backend/Backend.cpp
  include/backend/backend/FactorGraphOptimization.cpp
  include/backend/backend/GnssProcessor.cpp
  include/backend/backend/LoopClosure.cpp
  include/backend/backend/Relocalization.cpp
  include/backend/utility/Pcd2Pgm.cpp
  include/backend/global_localization/scancontext/Scancontext.cpp
  include/backend/global_localization/InitCoordinate.cpp)

file(GLOB_RECURSE LIO_LIB_SRC "src/lio/*.cpp" "src/ros/*.cpp")
file(GLOB_RECURSE LIO_LIB_HEADERS "include/lio/*.hpp" "include/lio/*.h")

add_library(lio ${LIO_LIB_SRC} ${LIO_LIB_HEADERS} ${BACKEND_SOURCES})

ament_target_dependencies(lio PUBLIC
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  nav_msgs
  pcl_ros
  livox_ros_driver2
  basic
)

target_link_libraries(lio PUBLIC
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  Boost::filesystem
  gtsam
  stdc++fs
  glog
  pthread
  cpu_bbs3d
  OpenMP::OpenMP_CXX
  ${cpp_typesupport_target}
)
if(BUILD_CUDA)
  target_link_libraries(lio PUBLIC gpu_bbs3d)
endif()

add_executable(super_lio_node src/apps/super_lio_node.cpp)
target_link_libraries(super_lio_node
  lio
  Boost::filesystem
  ${cpp_typesupport_target}
)

add_executable(relocation_node src/apps/relocation_node.cpp)
target_link_libraries(relocation_node
  lio
  Boost::filesystem
  ${cpp_typesupport_target}
)

add_executable(status_switch src/apps/status_switch.cpp)
ament_target_dependencies(status_switch rclcpp std_msgs)

install(TARGETS
  lio
  cpu_bbs3d
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
if(BUILD_CUDA)
  install(TARGETS gpu_bbs3d)
endif()

install(TARGETS
  super_lio_node status_switch
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  relocation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  3rdparty/
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
