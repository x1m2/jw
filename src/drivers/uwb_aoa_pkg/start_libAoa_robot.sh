uart_name="/dev/ttyUSB0"

sudo chmod 666 $uart_name

colcon build
source ./install/setup.bash

ros2 run uwb_aoa_pkg libAoa_robot_example $uart_name
