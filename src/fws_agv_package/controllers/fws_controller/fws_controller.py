"""fws_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Keyboard

# create the Robot instance.
robot = Robot()
keyboard = Keyboard()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
ps =[]
pm=[]
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
steers =['left steering motor','right steering motor','left assist steering motor','right assist steering motor']
motors = ['left drive wheel motor','right drive wheel motor']
sensors = ['left drive position sensor', 'right drive position sensor','left steering sensor','right steering sensor','left assist steering sensor','right assist steering sensor']

st = [robot.getDevice(steer) for steer in steers]
pm = [robot.getDevice(motor) for motor in motors]
ps = [robot.getDevice(sensor) for sensor in sensors]

[p.enable(timestep) for p in ps]
keyboard.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    
    val = [p.getValue() for p in ps]
    print(f'Drive: {val[0]},{val[1]}\nSteer: {val[2]},{val[3]},{val[4]},{val[5]}')
    # Process sensor data here.
    key=keyboard.getKey()

    if (key==ord('Q')):
        for s in st:
          s.setVelocity(100)
          s.setPosition(0.7) 
          
    elif (key==ord('E')):
      for s in st:
          s.setVelocity(100)
          s.setPosition(-0.7) 
          
    # Enter here functions to send actuator commands, like:
    elif (key==ord('W')):
       for motor in pm:
            motor.setPosition(float('inf'))
            motor.setVelocity(10)
    elif (key==ord('S')):
        for motor in pm:
            motor.setPosition(float('inf'))
            motor.setVelocity(-10)
    elif (key==ord('R')):
      for s in st:
          s.setPosition(0) 
          s.setVelocity(10)
      for motor in pm:
            motor.setPosition(float('inf'))
            motor.setVelocity(0)
    pass

# Enter here exit cleanup code.
