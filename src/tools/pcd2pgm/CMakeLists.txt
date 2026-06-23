cmake_minimum_required(VERSION 3.8)
project(pcd2pgm)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)

###########
## Build ##
###########

# 创建库
ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
)

# 注册组件并创建可执行文件
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN pcd2pgm::Pcd2PgmNode
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Install ##
#############

# 使用 ament_auto_install 自动处理所有安装
ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
