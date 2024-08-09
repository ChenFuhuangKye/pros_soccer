#!/bin/bash
source /opt/ros/humble/setup.bash

colcon build 
source /workspaces/src/install/setup.bash

ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ros2 run pros_yolo soccer_node &
ros2 run pros_soccer car_control

# 防止腳本結束導致容器退出
wait