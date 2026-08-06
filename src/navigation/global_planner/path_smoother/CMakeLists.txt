cmake_minimum_required(VERSION 3.8)
project(path_smoother)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(navigation_interfaces REQUIRED)
find_package(robot_runtime REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}
  src/path_smoother.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  navigation_interfaces
  nav_msgs
  geometry_msgs
  visualization_msgs
)

add_executable(${PROJECT_NAME}_node
  src/path_smoother_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  robot_runtime
  navigation_interfaces
)

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
    FILES_MATCHING PATTERN "*.launch.py"
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  navigation_interfaces
  nav_msgs
  geometry_msgs
  visualization_msgs
)

ament_package()
