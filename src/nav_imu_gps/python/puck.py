#!/bin/env python3
import serial.tools.list_ports as usbs
import utm
import sys
from serial import *

"""
Author: Yash Mewada
Created: Jan 25' 2023

"""

class BU_353S4:

    def __init__(self,port_arg):
        if sys.platform.startswith('win') or sys.platform.startswith('linux') or sys.platform.startswith('cygwin') or sys.platform.startswith('darwin'):
            self.ports = list(usbs.comports())
        else:
            raise EnvironmentError('Unsupported platform')
        self.attach_gps(port_arg)
        self.long_chars = ['E','W']
        self.lat_chars = ['S','N']
    
    def attach_gps(self,port_arg):
        """
        Args:
            port_arg: The USB port passed manually or automatic port selection.
        returns:
            None. Sets the port automatic or manual as per the argument.
        """
        expected_gps = "Prolific Technology Inc."
        gps_type = "USB-Serial Controller D"
        if port_arg == None:
            for p in self.ports:
                if (p.manufacturer == expected_gps and p.product == gps_type) or p.manufacturer == "Prolific":
                    port_arg = p.device
                    break
                else:
                    print("[-] Oops! it seems I cannot find the Puck!")
                    return
            try:
                self.gps = Serial(port_arg,timeout=None,baudrate=4800,xonxoff=False,rtscts=False,dsrdtr=False)
                self.gps.flush()
                self.gps.reset_input_buffer()
                self.gps.reset_output_buffer()
            except SerialException as e:
                print("[*] Please check the serial ports and connections ",e)
        else:
            self.gps = Serial(port_arg,timeout=None,baudrate=4800,xonxoff=False,rtscts=False,dsrdtr=False)
            self.gps.flush()
            self.gps.reset_input_buffer()
            self.gps.reset_output_buffer()
        
    def read_lat_long(self,data):
        """
        Args:
            data: parsed and formated data string
        returns:
            latitude and longitude converted into degree.minutes
        """
        if data[0] not in ('GPGGA','GPRMC') or not data[3]:
            return (0.0,0.0)
        lat_idx = [data.index(c) for c in self.lat_chars if c in data]
        lat_sign = 1 if data[lat_idx[0]] != self.lat_chars[0] else -1
        lat_idx = lat_idx[0] - 1

        long_idx = [data.index(c) for c in self.long_chars if c in data]
        long_sign = -1 if data[long_idx[0]] == self.long_chars[1] else 1
        long_idx = long_idx[0] - 1

        lat_ddmm = float(data[lat_idx][:2]) + float(data[lat_idx][2:])/60
        lon_ddmm = float(data[long_idx][:3]) + float(data[long_idx][3:])/60
        """
        Note there is change in string parsing and unpacking because the decimal degree for 
        Latitude ranges from -90 to 90 and that for Longitude ranges from -180 to 180.
        """
        return (lat_sign * lat_ddmm,long_sign * lon_ddmm)

    def read_utm_latlon(self,data):
        """
        Args:
            data: parsed and formated data string
        returns:
            UTM cooridnates from latitude and longitude
        """
        lat_long = self.read_lat_long(data)
        if lat_long != (0.0, 0.0):
            utm_latlon = utm.from_latlon(*lat_long)
            return utm_latlon

    def read_hdop(self,data):
        """
        Args:
            data: parsed and formated data string
        returns:
            Horizontal Dilution of precision
        """
        if data[0] == 'GPGGA':
            return round(float(data[8]),2)
        elif data[0] == 'GPGSA':
            return round(float(data[16]),2)
        else:
            return 0.0
    
    def read_altitude(self,data):
        """
        Args:
            data: parsed and formated data string
        returns:
            current Altitude
        """
        if data[0] == 'GPGGA':
            return round(float(data[9]),2)
        else:
            return 0.0
    
    def read_time(self,data):
        """
        Args:
            data: parsed and formated data string
        returns:
            Current GMT time
        """
        # print(data)
        if data[0] in ('GPGGA','GPRMC'):
            return data[1]

    def read_data(self):
        """
        Args:
            None.
        returns:
            parsed and formated data string
        """
        msg = self.gps.readline().decode().replace("$","").replace("\n","").replace("\r","").split(",")
        msg.pop()
        return msg
    
    def read_line(self):
        """
        Args:
            None.
        returns:
            Raw string from the serial port.
        """
        return self.gps.readline()  

# if __name__ == "__main__":
#     my_gps = BU_353S4(None)
#     while True:
#         dt = my_gps.read_line()
#         xt = my_gps.read_lat_long(dt)
#         if xt != None:
#             print(dt)