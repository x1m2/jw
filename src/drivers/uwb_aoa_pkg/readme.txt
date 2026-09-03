1.安装

  1.1 安装ros2及uwb_aoa_pkg
  	以安装ros2 humble为例，该版本可运行平台为Ubuntu 22.04 / Windows 10/ Red Hat Enterprise Linux 8.
  	方式1.参考官方文档:
  		https://docs.ros.org/en/humble/Installation.html
  	方式2.使用鱼香ros，打开终端输入：
  		wget http://fishros.com/install -O fishros && . fishros
  		
  	ros2安装完毕后，将uwb_aoa_pkg文件夹放入创建的ros2_workspace/src/中

  1.2 安装依赖包
  	编译及运行使用的依赖包存在requirement.txt中，可使用rosdep安装。rosdep安装方式：
  		sudo apt install python3-rosdep
  	安装rosdep后，进行初始化和更新
  		sudo rosdep init
  		rosdep update
  	rosdep安装完成后，可在ros2_workspace下使用终端用以下指令安装所有依赖：
  		rosdep install --from-paths src/uwb_aoa_pkg -y --ignore-src --rosdistro humble
  		
  1.3 其他依赖
	定位结果画图使用plotjuggler,需要提前安装，安装方式 sudo apt install ros-humble-plotjuggler-ros
	编译默认使用colcon，若并未安装，安装方式 sudo apt install python3-colcon-common-extensions


2.编译和运行
把uwb_aoa_pkg文件夹放在workspace/src/中
串口名称默认 "/dev/ttyUSB0"，可在start_libAoa_robot.sh内改变
测试用例使用./start_libAoa_robot.sh，它进行编译，run publisher/service节点两个操作。

链接串口，启动对手件测距，运行
  source ./start_libAoa_robot.sh 
即可启动测试

start_libAoa_robot.sh内：
  ros2 run uwb_aoa_pkg libAoa_robot_example "/dev/ttyUSB0"开启名为"/uwb_robot_publisher"的节点，发布4个topic:
  
  -定位结果及置信度 "/libAoa_robot_publisher",
  -雷达模式结果 "/uwb_robot_alive_sentry_publisher",
  -心跳 "/uwb_robot_heart_publisher"。
  -定位原始数据 "/uwb_robot_pos_rawdata_publisher" (仅在未使用全迹b1蓝牙板，串口直连时发布数据)
  
    并订阅一个service:
    
  -"/uwb_robot_start_radar"
    该服务用于控制开启/关闭uwb雷达模式。
    若需开启雷达模式，在终端中运行：
    -ros2 service call /uwb_robot_start_radar std_srvs/srv/SetBool "{data: true}"
    若需关闭雷达模式，在终端中运行：
    -ros2 service call /uwb_robot_start_radar std_srvs/srv/SetBool "{data: false}"
  
  若需要简单观察定位结果，运行
  ros2 run plotjuggler plotjuggler  --layout ./libaoa_example_layout.xml
  （为了画图需在弹出窗口点击两次确认，一次确认使用topic subscriber，一次选择libaoa_robot_publisher节点画图）
  
  4个topic对应的msg定义在./msg文件夹内，包含：
  
  -"/libAoa_robot_publisher"：
	-定位极坐标下的r,a，
	-xy坐标下的x,y，
	-算法状态state,
	-uwb信号强度rssi_uwb，
	-蓝牙信号强度rssi_ble,
	-置信度pos_confidence
	
  -"/uwb_robot_alive_sentry_publisher"：
  	-是否有活体或触摸 is_have_living_body，
  		bit0为活体总结果，0：无活体，1：有活体
  		bit1为摸头结果，0：无摸头，1：有摸头
  		bit2-7保留
  		
  	-调试数据 max_radar_v,
  	-活体距离 living_body_first_index_dis
  		bit0有活体时参考该值，单位cm
  		bit1有摸头时忽略该值
  		
  -"/uwb_robot_heart_publisher"：
  	-心跳计数 heart_cnt,
  	-测距状态 range_status,
  		01-02保留
  		03停止测距
  		04保留
  		05开始测距
  		06测距超时，对端突然关闭或者远离导致
  		
  	-雷达状态 radar_status,
  		01-02保留
  		03停止雷达
  		04保留
  		05开始雷达
  		06雷达超时，跟测距冲突
  		
  -"/uwb_robot_pos_rawdata_publisher"：
  	-串口原始数据测距距离 distance
  	-串口原始数据角度数据 angle
  	-串口原始数据俯仰角 pitch
  	-串口原始数据uwb信号强度 rssi_uwb
  	-串口原始蓝牙强度 rssi_ble
  	-串口原始置信度 pos_confidence

3.代码路径
如果需要修改代码，可以参考libAoa_robot_example.cpp，以ARM平台举例，路径如下：
ros2_humble_aarch64\uwb_aoa_pkg\src\libAoa_robot_example.cpp
串口数据解析可参考uart_stack.c,路径如下：
ros2_humble_aarch64\uwb_aoa_pkg\src\uart_stack.c
对应头文件路径如下：
ros2_humble_aarch64\uwb_aoa_pkg\include
雷达模式开启/关闭指令可参考uart_stack.h内宏定义，路径如下：
ros2_humble_aarch64\uwb_aoa_pkg\include\uwb_aoa_pkg\uart_stack.h

