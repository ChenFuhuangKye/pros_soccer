#!/bin/bash
source /opt/ros/humble/setup.bash


ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ros2 run pros_yolo soccer_node

# 防止腳本結束導致容器退出
wait