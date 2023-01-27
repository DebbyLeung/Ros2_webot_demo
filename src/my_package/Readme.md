## Access ttyUSB
'''
sudo chmod 777 /dev/ttyUSB0
'''
## Publish velocity
'''
ros2 topic pub /cmd_vel geometry_msgs/Twist  "linear: { x: 0.1 }"
'''
## Create node according to id
'''
ros2 run my_package feetech_controller --ros-args --remap [old_node_name]:__node:=[node_name] -p id:=[device_id]
ros2 run my_package feetech_controller --ros-args --remap feetech_controller_1:__node:=feetech_controller_3 -p id:=3
'''

