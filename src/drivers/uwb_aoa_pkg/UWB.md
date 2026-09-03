
sudo usermod -aG dialout $USER

mkdir -p ~/ros2_ws/src
cp -r uwb_aoa_pkg ~/ros2_ws/src/



source /opt/ros/humble/setup.bash
cd ~/ros2_ws

# 编译
colcon build --packages-select  uwb_aoa_pkg
source install/setup.bash

# 验证
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run uwb_aoa_pkg libAoa_robot_example /dev/ttyUSB0
