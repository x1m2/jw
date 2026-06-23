cmake_minimum_required(VERSION 3.10.0)
project(nano_gicp)

# 1. 设置 C++ 标准 (ROS 2 推荐 C++ 17)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

### set compiler
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions -g -ggdb")

# Find OpenMP
find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra ${CMAKE_CXX_FLAGS}")

### 2. 获取依赖包
# 替换 catkin 为 ament_cmake
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

# 3. 定义库
add_library(nanoflann src/nanoflann.cc)
# 修正：PCL 在较新版本中推荐使用其定义的宏或直接链接其 target
target_link_libraries(nanoflann ${PCL_LIBRARIES})

add_library(nano_gicp src/nano_gicp.cc)
# ROS 2 中通常直接链接到 OpenMP::OpenMP_CXX 更加现代
target_link_libraries(nano_gicp 
  nanoflann
  ${PCL_LIBRARIES} 
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
)

# 4. 设置头文件路径 (现代 CMake 推荐使用 target_include_directories)
target_include_directories(nanoflann PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_include_directories(nano_gicp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# 5. 安装与导出 (ROS 2 必须步骤)
install(
  TARGETS nanoflann nano_gicp
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

# 6. Ament 宏
ament_export_include_directories(include)
ament_export_libraries(nano_gicp nanoflann)
ament_export_targets(export_${PROJECT_NAME})
ament_package()