cmake_minimum_required(VERSION 3.8)
project(robot_runtime_test)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  navigation_workflow_gui.py
  robot_runtime_test_gui.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
