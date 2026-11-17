cmake_minimum_required(VERSION 3.10)
project(quatro)

### set compiler
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-fexceptions -g -ggdb")

include(FindOpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
  message(FATAL_ERROR "OpenMP could not be found.")
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall ${CMAKE_CXX_FLAGS}")

### dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(teaserpp REQUIRED)

### TBB option
if(QUATRO_TBB)
  message(WARNING "Quatro with TBB")
  find_library(TBB_LIBRARY tbb)
  message(WARNING "TBB FOUND, ${TBB_LIBRARY}")
  add_definitions(-DTBB_EN)

  include(ProcessorCount)
  ProcessorCount(N)
  message(WARNING "Processor number: ${N}")
  if(N GREATER 4)
    add_definitions(-DTBB_PROC_NUM=${N})
  else()
    add_definitions(-DTBB_PROC_NUM=1)
  endif()
endif()

### debug option
if(QUATRO_DEBUG)
  add_definitions(-DQUATRO_DEBUG)
endif()

### include dirs
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

### build library
add_library(${PROJECT_NAME}
  src/fpfh.cc
  src/matcher.cc
  src/quatro_module.cc
)

### link libraries
if (QUATRO_TBB)
  target_link_libraries(${PROJECT_NAME}
    ${OpenMP_LIBS}
    ${PCL_LIBRARIES}
    teaserpp::teaser_registration
    teaserpp::teaser_io
    ${TBB_LIBRARY}
  )
else()
  target_link_libraries(${PROJECT_NAME}
    ${OpenMP_LIBS}
    ${PCL_LIBRARIES}
    teaserpp::teaser_registration
    teaserpp::teaser_io
  )
endif()

### install（ROS2必须）
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/
)

### export
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(Eigen3 PCL teaserpp)

### finalize
ament_package()