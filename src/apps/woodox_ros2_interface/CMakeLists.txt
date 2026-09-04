cmake_minimum_required(VERSION 3.8)
project(woodox_ros2_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/GetUp.srv"
    "srv/GetDown.srv"
    "srv/Damp.srv"
    "srv/Ready.srv"
    "srv/Move.srv"
    "srv/GetMode.srv"
    "srv/SetMode.srv"
    "srv/GetState.srv"
    "srv/Crawl.srv"
    "msg/IMU.msg"
    "msg/BmsState.msg"
    "msg/BmsStates.msg"
    "msg/MotorState.msg"
    "msg/MotorStates.msg"
    "msg/WheelState.msg"
    "msg/MotorCmd.msg"
    "msg/MotorCmds.msg"
    DEPENDENCIES std_msgs
    )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
