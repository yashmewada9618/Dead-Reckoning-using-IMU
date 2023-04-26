from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import math
import numpy as np

class bag_file_reader:
    def __init__(self,bag_file):
        my_bag = bagreader(bag_file)
        self.myImu = my_bag.message_by_topic('/imu')
        self.myGps = my_bag.message_by_topic('/gps')

    def cvt_bag_csv(self):
        dataImu = pd.read_csv(self.myImu)
        dataGps = pd.read_csv(self.myGps)
        return dataImu
    
    def get_gyros(self):
        data = pd.read_csv(self.myImu,usecols = ['header.stamp.secs','angular_velocity.x','angular_velocity.y','angular_velocity.z'])

        data['time'] = data['header.stamp.secs']- data['header.stamp.secs'][0]
        data['gyro_x'] = data['angular_velocity.x']
        data['gyro_y'] = data['angular_velocity.y']
        data['gyro_z'] = data['angular_velocity.z']
        
        mean_x = np.mean(data['gyro_x'] )
        median_x = np.median(data['gyro_x'] )
        # print("mean gyro x: ", mean_x, end=" ")
        # print("median gyro x: ",median_x)
        # print("\n")

        mean_y = np.mean(data['gyro_y'] )
        median_y = np.median(data['gyro_y'] )
        # print("mean gyro y: ", mean_y, end=" ")
        # print("median gyro y: ",median_y)
        # print("\n")

        mean_z = np.mean(data['gyro_z'])
        median_z = np.median(data['gyro_z'])
        # print("mean gyro z: ", mean_z, end=" ")
        # print("median gyro z: ",median_z)
        # print("\n")

        # concatenated = pd.concat([data['angular_velocity.x'],data['angular_velocity.y'],data['angular_velocity.z']])
        sea.lineplot(x=data['time'], y= data['gyro_x'], data=data,label = 'Gyro x(rad/s)')
        sea.lineplot(x=data['time'], y= data['gyro_y'], data=data,label = 'Gyro y(rad/s)')
        sea.lineplot(x=data['time'], y= data['gyro_z'], data=data,label = 'Gyro z(rad/s)')
        plt.ylim(-0.0035,0.0035)
        plt.xlabel('Time (s)',fontsize=20)
        plt.ylabel('Gyro (rad/s)',fontsize=20)
        plt.title('Gyro vs Time',fontsize=20)
        filename = 'plot-' + str('Gyro_x vs Time') +'.png'
        plt.savefig(filename)
        return mean_x,median_x,mean_y,median_y,mean_z,median_z

    def get_accels(self):
        data = pd.read_csv(self.myImu,usecols = ['header.stamp.secs','linear_acceleration.x','linear_acceleration.y','linear_acceleration.z'])

        data['time'] = data['header.stamp.secs']- data['header.stamp.secs'][0]
        data['accel_x'] = data['linear_acceleration.x']
        data['accel_y'] = data['linear_acceleration.y']
        data['accel_z'] = data['linear_acceleration.z']
        
        sea.lineplot(x=data['time'], y= data['accel_x'], data=data,label = 'accel x(m/s^2)')
        sea.lineplot(x=data['time'], y= data['accel_y'], data=data,label = 'accel y(m/s^2)')
        sea.lineplot(x=data['time'], y= data['accel_z'], data=data,label = 'accel z(m/s^2)')
        # sea.histplot(data['accel_x'],bins=50)
        # plt.ylim(-0.09,0.09)
        plt.xlabel('Time(s)',fontsize=20)
        plt.ylabel('Accel (m/s^2)',fontsize=20)
        plt.title('Accel vs Time',fontsize=20)
        filename = 'plot-' + str('Accel vs Time') +'.png'
        plt.savefig(filename)

    def get_gyor_hist(self):
        data = pd.read_csv(self.myImu,usecols = ['header.stamp.secs','angular_velocity.x','angular_velocity.y','angular_velocity.z'])

        data['time'] = data['header.stamp.secs']- data['header.stamp.secs'][0]
        data['gyro_x'] = data['angular_velocity.x']
        sea.set_style("white")
        kwargs = dict(hist_kws={'alpha':.6}, kde_kws={'linewidth':2})
        plt.figure(figsize=(10,7), dpi= 80)
        sea.distplot(data['gyro_x'], color="dodgerblue", label="gyro_x", **kwargs)
        mean = np.mean(data['gyro_x'])
        median = np.median(data['gyro_x'])
        sd = np.std(data['gyro_x'])
        # sea.lineplot(x=data['time'], y= data['accel_x'], data=data,label = 'accel x(m/s^2)')
        # sea.histplot(data['accel_x'],bins=50)
        # plt.ylim(-0.09,0.09)
        # plt.ylabel('Count',fontsize=20)
        # plt.xlabel('Accel (m/s^2)',fontsize=20)
        # plt.title('gyro_x histograph',fontsize=20)
        # plt.legend()
        # filename = 'plot-' + str('gyor_x hist') +'.png'
        # # (-0.3061888328809616, -0.306, 0.013776184675224222)
        # plt.savefig(filename)
        # return mean,median,sd

    def get_accel_hist(self):
        data = pd.read_csv(self.myImu,usecols = ['header.stamp.secs','linear_acceleration.x','linear_acceleration.y','linear_acceleration.z'])

        data['time'] = data['header.stamp.secs']- data['header.stamp.secs'][0]
        data['accel_x'] = data['linear_acceleration.x']
        sea.set_style("white")
        kwargs = dict(hist_kws={'alpha':.6}, kde_kws={'linewidth':2})
        plt.figure(figsize=(10,7), dpi= 80)
        sea.distplot(data['accel_x'], color="dodgerblue", label="Acceleration X", **kwargs)
        mean = np.mean(data['accel_x'])
        median = np.median(data['accel_x'])
        sd = np.std(data['accel_x'])
        # sea.lineplot(x=data['time'], y= data['accel_x'], data=data,label = 'accel x(m/s^2)')
        # sea.histplot(data['accel_x'],bins=50)
        # plt.ylim(-0.09,0.09)
        plt.ylabel('Count',fontsize=20)
        plt.xlabel('Accel (m/s^2)',fontsize=20)
        plt.title('Accel_x histograph',fontsize=20)
        plt.legend()
        # filename = 'plot-' + str('Accel_x hist') +'.png'
        # # (-0.3061888328809616, -0.306, 0.013776184675224222)
        # plt.savefig(filename)
        # return mean,median,sd
    

    def to_eulers(self,w,x,y,z):
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = math.sqrt(1 + 2 * (w * y - x * z))
        cosp = math.sqrt(1 - 2 * (w * y - x * z))
        pitch = 2 * math.atan2(sinp, cosp) - math.pi / 2

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return math.degrees(yaw),math.degrees(pitch),math.degrees(roll)

    def get_ypr(self):
        data = pd.read_csv(self.myImu,usecols = ['Header.stamp.secs','IMU.orientation.x','IMU.orientation.y','IMU.orientation.z','IMU.orientation.w'])
        data['time'] = data['Header.stamp.secs'] - data['Header.stamp.secs'][0]
        # data['s_x'] = data['orientation.x']- data['orientation.x'][0]
        # data['s_y'] = data['orientation.y']- data['orientation.y'][0]
        # data['s_z'] = data['orientation.z']- data['orientation.z'][0]
        # data['s_w'] = data['orientation.w']- data['orientation.w'][0]
        # print(data)
        eul_x = []
        eul_y = []
        eul_z = []
        time = []
        for i in range(365,600):
            time.append(float(data.iat[i,0]) - data['Header.stamp.secs'][0])
            x = float(data.iat[i,1])
            y = float(data.iat[i,2])
            z = float(data.iat[i,3])
            w = float(data.iat[i,4])
            yaw,pitch,roll = self.to_eulers(w,x,y,z)
            eul_x.append(roll)
            eul_y.append(pitch)
            eul_z.append(yaw)
        plt.plot(time,eul_x,label = "Roll")
        plt.plot(time,eul_y,label = "Pitch")
        plt.plot(time,eul_z,label = "Yaw")
        # plt.legend()
        plt.ylabel('Euler (deg)',fontsize=20)
        plt.xlabel('Time (s)',fontsize=20)
        plt.title('Euler vs Time',fontsize=20)
        plt.legend()
        filename = 'plot-' + str('Euler vs Time stat__yaw_pitch') +'.png'
        plt.savefig(filename)
        # plt.ylim()
            # y = 
            # z = 
            # w = 
            # eul_x.append(str(self.a.iat[i,1]).split(',')[10])
            # eul_y.append(str(self.a.iat[i,1]).split(',')[11])
            # eul_z.append(str(self.a.iat[i,1]).replace('*',",").split(',')[12])
        # print(data[0])
        # def video_script():
        #     data = pd.read_csv(self.myImu,usecols = ['header.stamp.secs','imu.x','imu.y','imu.z','imu.w'])

    


    
if __name__ == '__main__':
    bag_file_video = '/home/yash/Desktop/Lab_4/EECE5554/LAB4/src/Data/data_driving.bag'
    imu_bag = bag_file_reader(bag_file_video)
