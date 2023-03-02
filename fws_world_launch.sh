#!/bin/sh
cd ~/Ros2_webot_demo
colcon build
source install/local_setup.bash
# ros2 launch my_package world_launch.py
