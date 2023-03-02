# Simulation model of 2 wheel steering robot
## To start a simulation 
```
ros2 launch fws_agv_package world_launch.py
```
with a new terminal
```
ros2 run fws_agv_package teleop_twist_keyboard
```
Show command velocity
```
ros2 topic echo /cmd_vel 
```
