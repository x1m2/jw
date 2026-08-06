cmake_minimum_required(VERSION 3.8)
project(rviz_topology_editor)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rviz_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TopologyEdgePolicy.msg"
  "msg/TopologyEdgePolicyArray.msg"
  "srv/SetTopologyEdgePolicy.srv"
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(rviz_topology_editor_node
  src/rviz_topology_editor_node.cpp
  src/main.cpp
)

target_include_directories(rviz_topology_editor_node PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(rviz_topology_editor_node
  ${PCL_LIBRARIES}
  "${cpp_typesupport_target}"
)

ament_target_dependencies(rviz_topology_editor_node
  geometry_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_srvs
  visualization_msgs
)

add_library(rviz_topology_editor_save_panel SHARED
  include/rviz_topology_editor/save_panel.hpp
  src/save_panel.cpp
)

set_target_properties(rviz_topology_editor_save_panel PROPERTIES AUTOMOC ON)

target_include_directories(rviz_topology_editor_save_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(rviz_topology_editor_save_panel
  Qt5::Widgets
  "${cpp_typesupport_target}"
)

ament_target_dependencies(rviz_topology_editor_save_panel
  pluginlib
  rclcpp
  rviz_common
  std_srvs
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(
  TARGETS
    rviz_topology_editor_node
    rviz_topology_editor_save_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch msg rviz srv
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
