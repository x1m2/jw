cmake_minimum_required(VERSION 3.16)
project(bbs3d LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
option(BUILD_OPENCL "Build the dense OpenCL backend" ON)
option(BUILD_BBS_TEST_TOOLS "Build standalone localization and voxel map tools" ON)
option(BUILD_BBS_OPENCL_TEST "Build the standalone OpenCL benchmark" OFF)
include(CMakePackageConfigHelpers)

find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

add_library(cpu_bbs3d SHARED
  bbs3d/src/cpu_bbs3d/bbs3d.cpp
  bbs3d/src/cpu_bbs3d/voxelmaps.cpp
  bbs3d/src/cpu_bbs3d/voxelmaps_io.cpp)
target_include_directories(cpu_bbs3d PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/bbs3d/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(cpu_bbs3d PUBLIC cxx_std_17)
target_link_libraries(cpu_bbs3d PUBLIC Eigen3::Eigen OpenMP::OpenMP_CXX
  PRIVATE Boost::filesystem Boost::system)
set_target_properties(cpu_bbs3d PROPERTIES INSTALL_RPATH "$ORIGIN")
install(TARGETS cpu_bbs3d EXPORT bbs3dTargets LIBRARY DESTINATION lib)
install(DIRECTORY bbs3d/include/ DESTINATION include PATTERN opencl_bbs3d EXCLUDE)

if(BUILD_OPENCL)
  if(EXISTS "/usr/hobot/lib/libOpenCL.so")
    set(OpenCL_LIBRARY "/usr/hobot/lib/libOpenCL.so" CACHE FILEPATH "OpenCL runtime")
  endif()
  find_package(OpenCL REQUIRED)
  set(kernel_dir "${CMAKE_CURRENT_SOURCE_DIR}/bbs3d/src/opencl_bbs3d")
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${kernel_dir}/score.cl")
  file(READ "${kernel_dir}/score.cl" DENSE_SCORE_CL)
  configure_file("${kernel_dir}/score_source.hpp.in" generated/score_source.hpp @ONLY)

  add_library(opencl_bbs3d SHARED bbs3d/src/opencl_bbs3d/bbs3d.cpp)
  target_include_directories(opencl_bbs3d PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated")
  target_link_libraries(opencl_bbs3d PUBLIC cpu_bbs3d PRIVATE OpenCL::OpenCL)
  get_filename_component(opencl_runtime_dir "${OpenCL_LIBRARY}" DIRECTORY)
  set_target_properties(opencl_bbs3d PROPERTIES
    INSTALL_RPATH "$ORIGIN;${opencl_runtime_dir}")
  install(TARGETS opencl_bbs3d EXPORT bbs3dTargets LIBRARY DESTINATION lib)
  install(DIRECTORY bbs3d/include/opencl_bbs3d DESTINATION include)
endif()

configure_package_config_file(cmake/bbs3dConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/bbs3dConfig.cmake"
  INSTALL_DESTINATION lib/cmake/bbs3d)
install(EXPORT bbs3dTargets NAMESPACE bbs3d:: DESTINATION lib/cmake/bbs3d)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/bbs3dConfig.cmake"
  DESTINATION lib/cmake/bbs3d)

if(BUILD_BBS_TEST_TOOLS)
  add_subdirectory(test)
endif()
