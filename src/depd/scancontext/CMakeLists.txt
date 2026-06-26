cmake_minimum_required(VERSION 3.16)
project(scancontext LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  include/scancontext
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED src/Scancontext.cpp)
ament_target_dependencies(${PROJECT_NAME}
  pcl_conversions
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
