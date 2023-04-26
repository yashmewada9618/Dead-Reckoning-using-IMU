# Dead-Reckoning-using-IMU
This repo  contains the trajectory estimation using IMU and is compared with the GPS trajectory
A ROS driver previous developed [here](https://github.com/yashmewada9618/ROS-navigation-stack-using-IMU-and-RTK-GPS) was used.
Steps performed here are:
- Magnetometer Calibration (Soft and Hard Iron)
- Estimating heading (Yaw) from Gyro and Magentometer
- Complementary Filter (Combination of High Pass and Low Pass filter) on Yaw from Gyro and magnetometer
- Estimating forward velocity from accelerometer
- Estimating the Center Of Mass of the vehicle based on the IMU's Linear and angular velocities.
