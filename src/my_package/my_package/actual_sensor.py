import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class ActualSensor(Node):
    def __init__(self):
        super().__init__('actual_sensor')
        self.__publisher = self.create_publisher(Range, 'read_sensor', 1)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    
    def timer_callback(self):
        cmd = Range()
        cmd.range = 1.7777 # actual distance   
        self.__publisher.publish(cmd)
        self.i += 1    

        
def main(args=None):
    rclpy.init(args=args)
    sensor = ActualSensor()
    rclpy.spin(sensor)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sensor.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()