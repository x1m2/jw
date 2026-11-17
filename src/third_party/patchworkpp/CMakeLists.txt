cmake_minimum_required(VERSION 3.8)
project(patchworkpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

add_library(ground_seg_common STATIC
  common/src/plane_fit.cpp
)
set_target_properties(ground_seg_common PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(ground_seg_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/common/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ground_seg_common Eigen3::Eigen)

add_library(patchworkpp_core STATIC
  src/patchworkpp.cpp
)
set_target_properties(patchworkpp_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(patchworkpp_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/common/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(patchworkpp_core
  Eigen3::Eigen
  ground_seg_common
)

add_library(${PROJECT_NAME}::patchworkpp_core ALIAS patchworkpp_core)

install(DIRECTORY include/ common/include/
  DESTINATION include
)

install(TARGETS ground_seg_common patchworkpp_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(ground_seg_common patchworkpp_core)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3
  PCL
  rclcpp
  sensor_msgs
  pcl_conversions
)

ament_package()
