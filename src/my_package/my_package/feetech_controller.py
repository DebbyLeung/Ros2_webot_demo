import struct
from ctypes import *
import serial
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
class FunctionId():
    rc =0x01  # 00001-0ffff
    wc = 0x05 # 00001-0ffff

    rin =0x02 # 10001-1ffff
    win =0x04 # 30001-3ffff

    rreg = 0x03 # 40001-4ffff
    wreg = 0x06 # 40001-4ffff

class FeetechController(Node):
    def __init__(self,ser: serial.Serial) -> None:
        node_name = 'feetech_controller_1'
        super().__init__(node_name)
        self.declare_parameter('id', 1)
        self.device_id :int = self.get_parameter('id').get_parameter_value().integer_value

        self.ser = ser
        self.create_subscription(Twist, 'cmd_vel', self.__follow_motor, 1)
    def __follow_motor(self,msg:Twist):
        target_vel = min(msg.linear.x,10)
        self.set_vel(id,target_vel)
    def print_cmd(self,cmd):
        print(f"{cmd[3]+1}", end=":")
        for c in cmd:
            print(hex(c), end=" ")
        print()

    def crc16_generator_hex(self,b_data: list[int]) -> str:
        """CRC-16-MODBUS Hex Algorithm
        Parameters
        ----------
        data : list[int]
            Data packets received.
        Returns
        -------
        bytearray
            command append crc16
        Raises
        ----------
        ValueError
            If data packet in each index contains a byte > 256
        """
        cmd = bytearray(b_data)
        crc = 0xFFFF
        
        # Calculate CRC-16 checksum for data packet
        for b in b_data:
            crc ^= b
            for _ in range(0, 8):
                bcarry = crc & 0x0001
                crc >>= 1
                if bcarry:
                    crc ^= 0xa001
        b_crc = crc.to_bytes(2, 'little')
        cmd.extend(b_crc) 
        return cmd

    def read_reg(self, addr_start:int,addr_end:int):
        if addr_end<addr_start:
            print("address length is less than 1")
            return 0
        b_data = struct.pack('>BBHH', self.device_id,FunctionId.rreg,addr_start-1,addr_end-addr_start+1)
        cmd = self.crc16_generator_hex(b_data)
        self.ser.write(cmd)
        recv = bytearray(self.ser.read(8))
        return recv
    def write_reg(self, addr_start:int,values:c_uint8):
        b_data = struct.pack('>BBHH', self.device_id,FunctionId.wreg,addr_start-1,values)
        cmd = self.crc16_generator_hex(b_data)
        # self.print_cmd(cmd)
        self.ser.write(cmd)
        time.sleep(0.1)   
    def select_mode(self,mode:int):
        """select mode (0x10)
        Args:
            id (int): device id
            mode (int): servo mode-1, velocity mode-2
        """
        if mode == 0:
            self.write_reg(17,0)
        elif mode == 1:    
            self.write_reg(17,1)
    def set_vel(self,id:int,vel:float):
        """set motor velocity(0x83)

        Args:
            id (int): device id
            vel (float): velocity in deg/s
        """
        VEL_MAX = 65535
        VEL_MIN = 0
        vel_ = max(min(vel*65536/360,VEL_MAX),VEL_MIN)  
        print(f"vel: {vel_} counts")
        self.write_reg(132,int(vel_))

    def set_pos(self, pos:float,vel:float=0.5):
        """set motor position(0x81) & velocity(0x83)

        Args:
            id (int): device id
            pos (float): position in deg
            vel (float, optional): velocity in deg/s. Defaults to 0.5.
        """
        POS_MAX = 4095
        POS_MIN = 0
        pos_ = max(min(pos*4096/360,POS_MAX),POS_MIN)
        print(f"pos: {pos_} counts")
        self.set_vel(self.device_id,vel)
        self.write_reg(129,int(pos_))  

    def motor_stop(self):
        """stop motor(0x81), disable torque

        Args:
            id (int): device id
        """
        self.write_reg(130,0)

def main(args=None):       
    rclpy.init(args=args)
    try:
        ser = serial.Serial("/dev/ttyUSB0",baudrate = 115200, timeout=2)
    except serial.SerialException as e:
        print(e)
    controller = FeetechController(ser)

    

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # controller1.destroy_node()

    
    try:
        rclpy.spin(controller)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        controller.destroy_node()
        rclpy.try_shutdown()
    
    
if __name__ == "__main__":
    # sudo chmod 777 /dev/ttyUSB0
    try:
        ser = serial.Serial("/dev/ttyUSB0",baudrate = 115200, timeout=2)
        controller = FeetechController(ser)
        controller.select_mode(2,0)
        controller.set_pos(2,0,0.5)
    except serial.SerialException as e:
        print(e) 

"""
read mode 02 03 00 10 00 01
17 select mode  02 06 00 10 00 00 xx xx |where data[5] =0 is servo mode
129 pos 02 06 00 80 00 00 xx xx
130 stop 02 06 00 81 00 00
132 vel 02 06 00 83 00 20 xx xx
"""