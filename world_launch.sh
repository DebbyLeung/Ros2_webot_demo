#!/bin/sh
cd ~/ros2_ws
colcon build
source install/local_setup.bash
ros2 launch my_package world_launch.py
