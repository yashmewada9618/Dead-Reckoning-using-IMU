#!/bin/env python3
import math
from serial import *
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Rotation

"""
Author: Yash Mewada
Created: Feb 25' 2023

"""

class VN_100:

    def __init__(self,port_arg):
        self.imu = Serial(port_arg,timeout=None,baudrate=115200)
        self.imu.flush()
        self.imu.reset_input_buffer()
        self.imu.reset_output_buffer()
        self.set_binary_conf()
    
    def read_data(self):
        """
        Args:
            None.
        returns:
            parsed and formated data string
        """
        msg = self.imu.readline().decode().replace("$","").replace("\n","").replace("\r","").replace("*",",").split(',')
        msg.pop()
        return msg
    
    def read_line(self):
        """
        Args:
            None.
        returns:
            Raw string from the serial port.
        """
        return self.imu.readline()
    
    def set_binary_conf(self):
        # commd1 = "$VNWRG,75,2,20,01,0029*XX\r\n"
        commd1 = "$VNWRG,75,2,20,15,0009,000C,0014*XX\r\n"
        self.imu.write(commd1.encode())
        self.imu.readline()
# 
        commd2 = "$VNWRG,06,14*XX\r\n"
        # commd2 = "$VNWRG,06,2*XX\r\n" #command to output on qtn register
        self.imu.write(commd2.encode())
        self.imu.readline()

    def get_quatrenions(self):
        data = self.read_data()
        yaw = float(data[1])
        pitch = float(data[2])
        roll = float(data[3])

        cr = math.cos(math.radians(roll * 0.5))
        sr = math.sin(math.radians(roll * 0.5))
        cp = math.cos(math.radians(pitch * 0.5))
        sp = math.sin(math.radians(pitch * 0.5))
        cy = math.cos(math.radians(yaw * 0.5))
        sy = math.sin(math.radians(yaw * 0.5))

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        # (-0.019348272765196246, 0.017302898512855904, 0.37588363404407865, 0.9263032698322266)
        # -0.020245, +0.017897, +0.376158, +0.926162
        
        return x,y,z,w

if __name__ == "__main__":
    port = '/dev/ttyUSB0'
    my_imu = VN_100(port)
    # Create a rotation object from Euler angles specifying axes of rotation
    
    # my_imu.read_line()
    # my_imu.set_binary_conf()
    # print(my_imu.read_line())

    # my_imu.set_binary_conf2()
    # print(my_imu.read_line())
    # ser = Serial('/dev/ttyUSB0', 115200, timeout=1, rtscts=True)

    # # Write data to the serial port
    # ser.write(b'-V\r\n')

    # # Read data from the serial port
    # data = ser.readline(10)
    # print(data)

    # Close the serial port
    # ser.close()
    # print(my_imu.read_data())
    # print()
    while True:
        dt = my_imu.read_data()
        yaw = float(dt[3])
        pitch = float(dt[2])
        roll = float(dt[1])
        rot = Rotation.from_euler('xyz', [yaw, pitch, roll], degrees=True)
    
    # Convert to quaternions and print
        rot_quat = rot.as_quat()
        print("scipy",rot_quat)
        print("manual",my_imu.get_quatrenions())
        # print()