cmake_minimum_required(VERSION 3.8)
project(robot_urdf_generator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(robot_urdf_generator src/main.cpp)
target_link_libraries(robot_urdf_generator yaml-cpp)

install(TARGETS robot_urdf_generator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
