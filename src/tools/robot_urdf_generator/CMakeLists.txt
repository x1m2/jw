cmake_minimum_required(VERSION 3.8)
project(robot_urdf_generator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(urdf_generator_node src/main.cpp)
target_link_libraries(urdf_generator_node yaml-cpp)

install(TARGETS urdf_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
