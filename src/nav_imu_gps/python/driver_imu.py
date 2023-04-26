#!/bin/env python3
import rospy
from nav_imu_gps.msg import Vectornav
from nav_imu_gps.srv import to_quaternion
import math
from serial import *

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

    def set_binary_conf(self):
        """
        Args:
            None.
        returns:
            Set the binary configuration of the sensor.
        """
        # commd1 = "$VNWRG,75,2,20,01,0029*XX\r\n"
        commd1 = "$VNWRG,75,2,40,15,0009,000C,0014*XX\r\n"
        self.imu.write(commd1.encode())
        self.imu.readline()
        commd2 = "$VNWRG,06,14*XX\r\n"
        # commd2 = "$VNWRG,06,2*XX\r\n" #command to output on qtn register
        self.imu.write(commd2.encode())
        self.imu.readline()
    
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
    
    def get_quatrenions(self,req):

        cr = math.cos(math.radians(req.roll * 0.5))
        sr = math.sin(math.radians(req.roll * 0.5))
        cp = math.cos(math.radians(req.pitch * 0.5))
        sp = math.sin(math.radians(req.pitch * 0.5))
        cy = math.cos(math.radians(req.yaw * 0.5))
        sy = math.sin(math.radians(req.yaw * 0.5))

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return w,x,y,z
    

class imu_ros():
    def __init__(self):
        rospy.init_node('imu_driver',anonymous = True)
        arg_port = rospy.get_param("/imu_driver/port")
        self.my_imu = VN_100(arg_port)
        self.msg = Vectornav()
        self.pub = rospy.Publisher('imu',Vectornav,queue_size = 10)
        self.rate = rospy.Rate(10)
        rospy.loginfo_once("[o] No argument passed, considering AUTO PORT SELECTION") if not arg_port else rospy.loginfo_once("[/] Is this the port you passed for your IMU-- %s",arg_port)
    
    def service_push(self):
        service = rospy.Service('convert_to_quaternion', to_quaternion, self.my_imu.get_quatrenions)
    
    def service_get(self,y,p,r):
        rospy.wait_for_service('convert_to_quaternion')
        imu_quat = rospy.ServiceProxy('convert_to_quaternion', to_quaternion)
        resp1 = imu_quat(y,p,r)
        return resp1.w,resp1.x,resp1.y,resp1.z

    def talker(self):
        self.service_push()
        while not rospy.is_shutdown():
            data = self.my_imu.read_data()
            if (len(data) >=13):
                # print(len(data))
                yaw = float(data[1])
                pitch = float(data[2])
                roll = float(data[3])
                # print(yaw)
                Qts = self.service_get(yaw,pitch,roll)
                # rospy.loginfo(Qts)

                mag_x = float(data[4])
                mag_y = float(data[5])
                mag_z = float(data[6])

                accel_x = float(data[7])
                accel_y = float(data[8])
                accel_z = float(data[9])

                gyro_x = float(data[10])
                gyro_y = float(data[11])
                gyro_z = float(data[12])

                self.msg.header
                self.msg.header.stamp.secs = rospy.Time.now().secs
                self.msg.header.frame_id = 'imu1_frame'
                self.msg.header.stamp.nsecs = rospy.Time.now().nsecs

                self.msg.imu.w = Qts[0]
                self.msg.imu.x = Qts[1]
                self.msg.imu.y = Qts[2]
                self.msg.imu.z = Qts[3]

                self.msg.angular_velocity.x = gyro_x
                self.msg.angular_velocity.y = gyro_y
                self.msg.angular_velocity.z = gyro_z

                self.msg.linear_acceleration.x = accel_x
                self.msg.linear_acceleration.y = accel_y
                self.msg.linear_acceleration.z = accel_z

                self.msg.mag_field.x = mag_x
                self.msg.mag_field.y = mag_y
                self.msg.mag_field.z = mag_z

                self.msg.raw_data = str(self.my_imu.read_line())
                self.pub.publish(self.msg)

if __name__ == '__main__':
    imu_ros = imu_ros()
    try:
        imu_ros.talker()
    except rospy.ROSInterruptException: pass    