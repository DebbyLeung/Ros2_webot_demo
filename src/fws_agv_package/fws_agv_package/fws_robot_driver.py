import rclpy
from geometry_msgs.msg import Twist

WHEEL_RADIUS = 0.0535
steers =['left steering motor','right steering motor','left assist steering motor','right assist steering motor']
motors = ['left drive wheel motor','right drive wheel motor']
sensors = ['left drive position sensor', 'right drive position sensor','left steering sensor','right steering sensor','left assist steering sensor','right assist steering sensor']


class MyRobotDriver:
    def init(self, webots_node, properties):
        print("initializing")
        self.__robot = webots_node.robot


        self.__motor = [self.__robot.getDevice(motor) for motor in motors]
        self.__steering = [self.__robot.getDevice(steer) for steer in steers]
        self.__sensor = [self.__robot.getDevice(sensor) for sensor in sensors]
        [sensor.enable(64) for sensor in self.__sensor]
        # # driving wheel
        for motor in  self.__motor:
            motor.setPosition(float('inf'))
            motor.setVelocity(0)
        # steering motor
        for steering in  self.__steering:
            steering.setPosition(0)
            steering.setVelocity(100)
            
        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('fws_agv_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
   
    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist
        # twist.angular.z
        actual_motor = "moving" #some func to move motor
        
    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0.1)
        
        forward_speed = self.__target_twist.linear.x
        angular_speed = min(max(self.__target_twist.angular.z,-1.57),1.57) # restrict angle input

        command_motor_forward_speed = forward_speed/ WHEEL_RADIUS

        [motor.setVelocity(command_motor_forward_speed) for motor in  self.__motor]
        [steering.setPosition(angular_speed) for steering in  self.__steering]