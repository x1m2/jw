cmake_minimum_required(VERSION 3.8)
project(topology_rviz_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_runtime REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common search io)

include_directories(
  include
)

qt5_wrap_cpp(MOC_FILES
  include/topology_rviz_plugin/topology_tool.hpp
)

add_library(${PROJECT_NAME} SHARED
  src/topology_tool.cpp
  ${MOC_FILES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pluginlib
  rviz_common
  rviz_rendering
  rviz_ogre_vendor
  geometry_msgs
  interactive_markers
  std_msgs
  visualization_msgs
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Widgets
  ${PCL_LIBRARIES}
)

add_executable(target_points_visualizer_node
  src/target_points_visualizer_node.cpp
)

ament_target_dependencies(target_points_visualizer_node
  rclcpp
  robot_runtime
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME} target_points_visualizer_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/topology_connect.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch rviz 
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  pluginlib
  rviz_common
  rviz_rendering
  rviz_ogre_vendor
  geometry_msgs
  interactive_markers
  std_msgs
  visualization_msgs
)

ament_package()
