#!/bin/env python3
# from bagpy import bagreader
import pandas as pd
import seaborn as sea
from mpl_toolkits.mplot3d import Axes3D
from scipy import linalg, integrate
from scipy.signal import butter,filtfilt
import matplotlib.pyplot as plt
import numpy as np

class yaw_estimate:
    def __init__(self,bag_file):
        # my_bag = bagreader(bag_file)
        # self.my_topic = my_bag.message_by_topic('/vectornav')
        self.my_topic = bag_file
        self.imu_data = '/home/yash/Desktop/Lab_4/EECE5554/LAB4/src/Data/data_driving/imu.csv'
        self.start = 2900
        self.stop = 4500

    def cvt_bag_csv(self):
        data = pd.read_csv(self.my_topic)
        return data
    
    def to_eulers(self,w,x,y,z):
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = np.sqrt(1 + 2 * (w * y - x * z))
        cosp = np.sqrt(1 - 2 * (w * y - x * z))
        pitch = 2 * np.arctan2(sinp, cosp) - np.pi / 2

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.unwrap(yaw),np.unwrap(pitch),np.unwrap(roll)
    
    def imu_yaw(self):
        imu_data = pd.read_csv(self.imu_data)
        yaw,_,_ = self.to_eulers(imu_data['imu.w'],imu_data['imu.x'],imu_data['imu.y'],imu_data['imu.z'])
        # yaw = yaw - yaw[0]
        imu_data['time'] = imu_data['header.stamp.secs'][4500:] - imu_data['header.stamp.secs'][4500]
        sea.lineplot(y=yaw[4500:] - yaw[4500],x=imu_data['time'][4500:],label = "Yaw from IMUs")
        plt.xlabel('Time (s)')
        plt.ylabel('Yaw (deg)')
        plt.legend()
        plt.grid()
        # plt.ylim(-0.5,0.5)
        plt.title('Yaw from IMU',fontsize=9)
        filename = 'plot-' + str('Yaw from IMU') +'.png'
        plt.savefig(filename)
        # plt.axis("scaled")
        plt.show()

    def from_mag(self):
        imu_all = pd.read_csv(self.imu_data)
        data = pd.read_csv(self.my_topic)

        yaw = pd.DataFrame(columns=['est_yaw','est_yaw_pre'])
        
        est_yaw = np.arctan2(data["calibY"][4500:],data["calibX"][4500:])
        
        yaw["est_yaw_pre"] = np.arctan2(imu_all["mag_field.y"][4500:],imu_all["mag_field.x"][4500:])
        
        sea.lineplot(y = np.degrees(np.unwrap(est_yaw)),x = data["time"][4500:] - data["time"][4500],label="Yaw from Mag after calib")
        sea.lineplot(y = np.degrees(np.unwrap(yaw["est_yaw_pre"])),x = data["time"][4500:] - data["time"][4500],label="Yaw from Mag before calib")
        plt.xlabel('Time (s)')
        plt.ylabel('Yaw (deg)')
        plt.legend()
        plt.grid()
        # plt.ylim(-0.5,0.5)
        plt.title('Effect of magnetometer calibration on Yaw estimation',fontsize=9)
        filename = 'plot-' + str('Yaw from magnetometer') +'.png'
        plt.savefig(filename)
        # plt.axis("scaled")
        plt.show()
        return np.unwrap(est_yaw)
    
    def gyro_integrate(self,gyro_csv):
        gyro_data = pd.read_csv(gyro_csv)
        time_step = gyro_data['header.stamp.secs'][4500:] - gyro_data['header.stamp.secs'][4500]
        # print(time_step.shape)
        # print(gyro_data['angular_velocity.z'][self.start:self.stop].mean)
        gyro_int = integrate.cumtrapz(gyro_data['angular_velocity.z'][4500:],dx=0.05,initial=-0.001218)
        # print(gyro_int)
        # gyro_intwt = integrate.cumtrapz(gyro_data['angular_velocity.z'])
        # sea.lineplot(y=np.degrees(gyro_int),x=time_step,label = "Yaw from Gyro")
        # sea.lineplot(y=gyro,x=time_step[:25333],label = "Yaw from Gyro_notime")
        # sea.lineplot(y=gyro_init,x=time_step[:25334],label = "Yaw from Gyroinit0")
        # plt.xlabel('Time (s)')
        # plt.ylabel('Yaw (deg)')
        # plt.legend()
        # plt.grid()
        # # plt.ylim(-0.5,0.5)
        # plt.title('Combined Yaw from Mag and Gyro',fontsize=20)
        # plt.legend()
        # plt.grid()
        # # plt.axis("scaled")
        # filename = 'plot-' + str('Combined Yaw from Mag and Gyro') +'.png'
        # plt.savefig(filename)
        # plt.show()
        return gyro_int
    
    def magLowPassFilter(self,magYaw):
        fs = 20
        fc = 0.05

        order = 3
        nqy = 0.5*fs
        cutoff = fc/nqy
        print(cutoff)
        b,a = butter(order,cutoff,btype='low',analog=False)

        yawFiltered = filtfilt(b,a,magYaw)
        t = np.arange(magYaw.shape[0])/fs
        # sea.lineplot(x=t,y=magYaw,label = 'Original_mag')
        # sea.lineplot(x=t,y=yawFiltered,label = 'Filtered_mag')
        # plt.xlabel("Time (s)")
        # plt.ylabel("Yaw (rad)")
        # plt.title("Low Pass filter on Yaw from Magnetometer")
        # plt.grid()
        # plt.legend()
        # filename = 'plot-' + str('mag_low_pass') +'.png'
        # plt.savefig(filename)
        # plt.show()
        return np.unwrap(yawFiltered) - np.unwrap(yawFiltered)[0]
    
    def gyroHighPassFilter(self,gyro_yaw):
        fs = 20
        fc = 0.0001

        order = 3
        nqy = 0.5*fs
        cutoff = fc/nqy
        print(cutoff)
        b,a = butter(order,cutoff,btype='high',analog=False)

        yawFiltered = filtfilt(b,a,gyro_yaw)
        t = np.arange(gyro_yaw.shape[0])/fs
        # sea.lineplot(x=t,y=gyro_yaw,label = 'Original_gyro')
        # sea.lineplot(x=t,y=yawFiltered - yawFiltered[0],label = 'Filtered_gyro')
        # plt.xlabel("Time (s)")
        # plt.ylabel("Yaw (rad)")
        # plt.title("High Pass filter in Yaw from Gyro")
        # plt.grid()
        # plt.legend()
        # filename = 'plot-' + str('gyro_high_pass') +'.png'
        # plt.savefig(filename)
        # plt.show()
        return np.unwrap(yawFiltered) - np.unwrap(yawFiltered)[0]
    
    def CPF(self,LPF,HPF):
        fs = 20
        alpha = 0.8
        CPF = alpha*HPF + (1-alpha) *LPF
        t = np.arange(CPF.shape[0])/fs
        # sea.lineplot(x=t,y=CPF - CPF[0],label = 'CPF')
        # # sea.lineplot(x=t,y=yawFiltered,label = 'Filtered')
        # plt.xlabel("Time (s)")
        # plt.ylabel("Yaw (rad)")
        # plt.title("Complementary filtered output")
        # plt.grid()
        # plt.legend()
        # filename = 'plot-' + str('complemetary_filter') +'.png'
        # plt.savefig(filename)
        # plt.show()
        return CPF


if __name__ == '__main__':
    # bag_file_path = '/home/yash/Desktop/Lab_4/EECE5554/LAB4/src/Data/data_driving.bag'
    csv_file_path = '/home/yash/Desktop/Lab_4/EECE5554/LAB4/src/Data/data_driving/imu.csv'
    mag_csv = '/home/yash/Desktop/Lab_4/EECE5554/LAB4/src/Data/just_mag_data.csv'
    calibrated_mag_csv = '/home/yash/Desktop/Lab_4/EECE5554/LAB4/src/Data/calib_mag_all.csv'
    # bag_file_new_path = '/home/yash/Desktop/my_lab2/EECE5554/LAB3/src/Data/stat_new.bag'
    # magCal = yaw_estimate(csv_file_path)
    yaw = yaw_estimate(calibrated_mag_csv)
    # magYaw = yaw.from_mag()
    yaw.imu_yaw()
#     gyroYaw = yaw.gyro_integrate(csv_file_path)
#     magLPF = yaw.magLowPassFilter(magYaw)
#     gyroHPF = yaw.gyroHighPassFilter(gyroYaw)
#     yaw.imu_yaw()
#     yaw.CPF(magLPF,gyroHPF)
