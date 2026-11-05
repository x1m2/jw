cmake_minimum_required(VERSION 3.8)
project(scan_planner)

# Avoid silently building the map kernel without optimization after an OTA
# build-type typo such as "Releas". Explicit Debug builds remain supported.
if(NOT CMAKE_CONFIGURATION_TYPES)
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  elseif(NOT CMAKE_BUILD_TYPE MATCHES "^(Debug|Release|RelWithDebInfo|MinSizeRel)$")
    message(FATAL_ERROR "Unsupported CMAKE_BUILD_TYPE='${CMAKE_BUILD_TYPE}'; use Release, RelWithDebInfo, MinSizeRel or Debug")
  endif()
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
# find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(navigation_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(slam_manager REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

set(ros_dependencies
  # cv_bridge  # Temporarily disabled with depth image input.
  ament_index_cpp geometry_msgs message_filters nav_msgs navigation_interfaces pcl_conversions
  rclcpp sensor_msgs std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros visualization_msgs)

# Compile all SCAN modules into the executable; OTA need not deploy separate
# plan_env/path_searching/bspline_opt/traj_utils shared libraries.
add_executable(scan_planner_node
  src/plan_manage/scan_planner_node.cpp
  src/plan_manage/scan_replan_fsm.cpp
  src/plan_manage/planner_manager.cpp
  src/plan_manage/closed_loop_controller.cpp
  src/plan_env/grid_map.cpp
  src/plan_env/raycast.cpp
  src/path_searching/dyn_a_star.cpp
  src/bspline_opt/uniform_bspline.cpp
  src/bspline_opt/bspline_optimizer.cpp
  src/bspline_opt/gradient_descent_optimizer.cpp
  src/traj_utils/planning_visualization.cpp
  src/traj_utils/polynomial_traj.cpp)
target_include_directories(scan_planner_node PRIVATE
  include ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(scan_planner_node ${ros_dependencies} slam_manager)
target_link_libraries(scan_planner_node Eigen3::Eigen ${PCL_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS scan_planner_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES launch/default.rviz DESTINATION share/${PROJECT_NAME}/rviz)
install(PROGRAMS scripts/keypoint_recorder.py scripts/reference_path_publisher.py DESTINATION lib/${PROJECT_NAME})


ament_package()
