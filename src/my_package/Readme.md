## Publish velocity
'''
ros2 topic pub /cmd_vel geometry_msgs/Twist  "linear: { x: 0.1 }"
'''
## Create node according to id
'''
ros2 run my_package feetech_controller --ros-args -p id:=3
'''
## Access ttyUSB
'''
sudo chmod 777 /dev/ttyUSB0
'''
