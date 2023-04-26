from bagpy import bagreader
import pandas as pd
import seaborn as sea
from mpl_toolkits.mplot3d import Axes3D
from scipy import linalg
import matplotlib.pyplot as plt
import numpy as np

class magCal:
    def __init__(self,bag_file):
        # my_bag = bagreader(bag_file)
        # self.my_topic = my_bag.message_by_topic('/vectornav')
        self.my_topic = bag_file
        self.imu_data = '/home/yash/Desktop/Lab_4/EECE5554/LAB4/src/Data/data_driving/imu.csv'

    def cvt_bag_csv(self):
        data = pd.read_csv(self.my_topic)
        return data
    
    def mag_data(self):
        a = pd.read_csv(self.my_topic)
        mag = pd.DataFrame(columns = ['magX', 'magY', 'magZ','time']) 
        mag['time'] = a['header.stamp.secs'][2900:4500] - a['header.stamp.secs'][2900]
        mag['magX'] = a['mag_field.x'][2900:4500]
        mag['magY'] = a['mag_field.y'][2900:4500]
        mag['magZ'] = a['mag_field.z'][2900:4500]
        mag.to_csv('just_mag_data.csv')
    
    def remove_offset(self):
        a = pd.read_csv(self.my_topic)
        magZ_mean = a['magZ'].mean()
        magY_mean = a['magY'].mean()
        magX_mean = a['magX'].mean()

        a["magZ_new"] = a['magZ'] - magZ_mean
        a["magY_new"] = a['magY'] - magY_mean
        a["magX_new"] = a['magX'] - magX_mean
        # ax = plt.axes(projection='3d')
        sea.scatterplot(y = a['magY'],x = a['magX'],label = "Uncalibrated data")
        # plt.xlabel('Mag_x (Gauss)')
        # plt.ylabel('Mag_y (Gauss)')
        # plt.grid()
        # # # plt.xlim(-0.5,0.5)
        # # # plt.ylim(-0.5,0.5)
        # plt.legend()
        # plt.title('Mag_x vs Mag_y',fontsize=20)
        # filename = 'plot-' + str('mag x vs y') +'.png'
        # plt.savefig(filename)
        # plt.close()
        # ax.set_zlabel('Z Label')
        # # u = np.linspace(0, 2 * np.pi, 100)
        # # v = np.linspace(0, np.pi, 100)
        # # x = np.outer(np.cos(u), np.sin(v))
        # # y = np.outer(np.sin(u), np.sin(v))
        # # z = np.outer(np.ones(np.size(u)), np.cos(v))
        # # ax.plot_wireframe(x, y, z, rstride=10, cstride=10, alpha=0.5)
        # # ax.plot_surface(x, y, z, alpha=0.3, color='b')
        # # concatenated = pd.concat([data['angular_velocity.x'],data['angular_velocity.y'],data['angular_velocity.z']])
        # # plt.plot(a['time'], a['magX_new'],label = 'Mag x(rad/s)')
        # # plt.plot(a['time'], a['magY_new'],label = 'Mag y(rad/s)')
        # # plt.plot(a['time'], a['magZ_new'],label = 'Mag z(rad/s)')
        # # plt.ylim(-0.035,0.035)
        # plt.xlabel('Time (s)',fontsize=20)
        # plt.ylabel('Mag',fontsize=20)
        # plt.title('mAG vs Time',fontsize=20)
        # plt.legend()
        # plt.show()/

        return a["magX"],a["magY"],a["magZ"]
    
    def fit_ellipse(self,magX,magY):
        D1 = np.vstack([magX**2, magX*magY, magY**2]).T
        D2 = np.vstack([magX, magY, np.ones(len(magX))]).T
        S1 = D1.T @ D1
        S2 = D1.T @ D2
        S3 = D2.T @ D2
        T = -np.linalg.inv(S3) @ S2.T
        M = S1 + S2 @ T
        C = np.array(((0, 0, 2), (0, -1, 0), (2, 0, 0)), dtype=float)
        M = np.linalg.inv(C) @ M
        eigval, eigvec = np.linalg.eig(M)
        con = 4 * eigvec[0]* eigvec[2] - eigvec[1]**2
        ak = eigvec[:, np.nonzero(con > 0)[0]]
        return np.concatenate((ak, T @ ak)).ravel()
    
    def get_ell_pts(self,coeffs):
        """

        Convert the cartesian conic coefficients, (a, b, c, d, e, f), to the
        ellipse parameters, where F(x, y) = ax^2 + bxy + cy^2 + dx + ey + f = 0.
        The returned parameters are x0, y0, ap, bp, e, phi, where (x0, y0) is the
        ellipse centre; (ap, bp) are the semi-major and semi-minor axes,
        respectively; e is the eccentricity; and phi is the rotation of the semi-
        major axis from the x-axis.

        """

        # We use the formulas from https://mathworld.wolfram.com/Ellipse.html
        # which assumes a cartesian form ax^2 + 2bxy + cy^2 + 2dx + 2fy + g = 0.
        # Therefore, rename and scale b, d and f appropriately.
        a = coeffs[0]
        b = coeffs[1] / 2
        c = coeffs[2]
        d = coeffs[3] / 2
        f = coeffs[4] / 2
        g = coeffs[5]

        den = b**2 - a*c
        if den > 0:
            raise ValueError('coeffs do not represent an ellipse: b^2 - 4ac must'
                             ' be negative!')

        # The location of the ellipse centre.
        x0, y0 = (c*d - b*f) / den, (a*f - b*d) / den

        num = 2 * (a*f**2 + c*d**2 + g*b**2 - 2*b*d*f - a*c*g)
        fac = np.sqrt((a - c)**2 + 4*b**2)
        # The semi-major and semi-minor axis lengths (these are not sorted).
        ap = np.sqrt(num / den / (fac - a - c))
        bp = np.sqrt(num / den / (-fac - a - c))

        # Sort the semi-major and semi-minor axis lengths but keep track of
        # the original relative magnitudes of width and height.
        width_gt_height = True
        if ap < bp:
            width_gt_height = False
            ap, bp = bp, ap

        # The eccentricity.
        r = (bp/ap)**2
        if r > 1:
            r = 1/r
        e = np.sqrt(1 - r)

        # The angle of anticlockwise rotation of the major-axis from x-axis.
        if b == 0:
            phi = 0 if a < c else np.pi/2
        else:
            phi = np.arctan((2.*b) / (a - c)) / 2
            if a > c:
                phi += np.pi/2
        if not width_gt_height:
            # Ensure that phi is the angle to rotate to the semi-major axis.
            phi += np.pi/2
        phi = phi % np.pi

        return x0, y0, ap, bp, e, phi
        # def fit_ellipse(self,magX,magY,magZ):
        #     a1 = magX ** 2
        #     a2 = magY ** 2
        #     # a3 = magZ ** 2
        #     # a4 = 2 * np.multiply(magY, magZ)
        #     # a5 = 2 * np.multiply(magX, magZ)
        #     a6 = 2 * np.multiply(magX, magY)
        #     a7 = 2 * magX
        #     a8 = 2 * magY
    #     # a9 = 2 * magZ
    #     a10 = np.ones(len(magX)).T
    #     D = np.array([a1,a2,a6,a7,a8,a10])
    #     # print(D.shape)
    #     C1 = np.array([[-1, 1,  0, 0],
    #                     [1, -1, 0, 0],
    #                     [0, 0,  -4, 0],
    #                     [0, 0,  0, -4]])
    #     # print(a1)
    #     S = np.matmul(D,D.T)
    #     print(S.shape)
    #     S11 = S[:3,:3]
    #     S12 = S[:3,3:]
    #     S21 = S[3:,:3]
    #     S22 = S[3:,3:]
    #     # print(S)
    #     #print(S21)
    #     tmp = np.matmul(np.linalg.inv(C1), S11 - np.matmul(S12, np.matmul(np.linalg.inv(S22), S21)))
    #     eigenValue, eigenVector = np.linalg.eig(tmp)
    #     u1 = eigenVector[:, np.argmax(eigenValue)]

    #     # Eqn 13 solution
    #     u2 = np.matmul(-np.matmul(np.linalg.inv(S22), S21), u1)

    #     # Total solution
    #     u = np.concatenate([u1, u2]).T
    #     # print()

    #     Q = np.array([[u[0], u[2],
    #                   [u[2], u[1]]]])

    #     n = np.array([[u[3]],
    #                   [u[4]],
    #                   [u[5]]])

    #     d = u[6]
    #     return Q, n, d

    def calibrate_mag(self,u):
        a = pd.read_csv(self.my_topic)
        x0, y0, ap, bp, e, phi = self.get_ell_pts(u)

        a["magX"] = a['magX'] - x0
        a["magY"] = a['magY'] - y0

        pre_rot = np.array(((np.cos(phi),np.sin(phi)), (-np.sin(phi),np.cos(phi))))
        
        post_rot = np.array(((np.cos(-phi),np.sin(-phi)), (-np.sin(-phi),np.cos(-phi))))

        print(post_rot.shape)
        mag = pd.DataFrame(columns = ['calibX', 'calibY', 'calibZ'],index=range(1600)) 
        nmag = pd.DataFrame(columns = ['calibX', 'calibY', 'time'],index=range(1600)) 
        # Q,n,d,_ = self.fit_ellipse(a["magX"],a["magY"],a["magZ"])
        # Qinv = np.linalg.inv(Q)
        # b = -np.dot(Qinv, n)
        # Ainv = np.real(1 / np.sqrt(np.dot(n.T, np.dot(Qinv, n)) - d) * linalg.sqrtm(Q))
        # # A = Ainv.inv()
        # print(a["magX"].shape[0])
        # x = []
        # y = []
        for i in range(a["magX"].shape[0]):
            xvec = [a["magX"][i],a["magY"][i]]
            # print("xvec",np.shape(xvec))
            rot_data = np.matmul(pre_rot,xvec)
            mag["calibX"][i] = rot_data[0]/(ap/bp)
            mag["calibY"][i] = rot_data[1]
            # mag["calibZ"][i] = hHat[2]
            # print()
        # print(mag["calibY"])
        # mag['calibX'] = pd.Series(x)
        # mag['calibY'] = pd.Series(y)
        for i in range(a["magX"].shape[0]):
            xvec = [mag["calibX"][i],mag["calibY"][i]]
            # print("xvec",xvec.shape)
            rot_data = np.matmul(post_rot,xvec)
            nmag["calibX"][i] = rot_data[0]
            nmag["calibY"][i] = rot_data[1]

        nmag["time"] = a["time"]
        nmag.to_csv('calib_mag.csv')
        # new_u = self.fit_ellipse(nmag["calibX"],nmag["calibY"])
        # _,_, ap1, bp1, e1, phi1 = self.get_ell_pts(new_u)
        # sea.scatterplot(y = mag["calibY"],x = mag["calibX"])
        t = np.linspace(0, 2*np.pi, 100)
        plt.plot( x0+ap*np.cos(t), y0+bp*np.sin(t),color = 'r')
        sea.scatterplot(y = nmag["calibY"],x = nmag["calibX"],label = "Calibrated data")
        plt.xlabel('calib_Mag_x (Gauss)')
        plt.ylabel('calib_Mag_y (Gauss)')
        plt.grid()
        plt.axis("scaled")
        plt.xlim(-0.5,0.5)
        # plt.ylim(-0.5,0.5)
        plt.title('Calibrated magnetometer data',fontsize=20)
        plt.legend()
        filename = 'plot-' + str('calibrated_mag') +'.png'
        plt.savefig(filename)
        plt.show()
        return mag["calibX"],mag["calibY"]
        # a['hX'] = np.matmul(Ainv,(a["magX"].T - b))
        # a['hy'] = a["magY"].T
        # a['hz'] = a["magZ"].T
        # print(mag["calibX"])
    
    def calibrateWholeData(self,u):
        a = pd.read_csv(self.imu_data)
        x0, y0, ap, bp, e, phi = self.get_ell_pts(u)

        a["mag_field.x"] = a['mag_field.x'] - x0
        a["mag_field.y"] = a['mag_field.y'] - y0

        pre_rot = np.array(((np.cos(phi),np.sin(phi)), (-np.sin(phi),np.cos(phi))))
        
        post_rot = np.array(((np.cos(-phi),np.sin(-phi)), (-np.sin(-phi),np.cos(-phi))))

        print(post_rot.shape)
        mag = pd.DataFrame(columns = ['calibX', 'calibY', 'calibZ'],index=range(25334))
        nmag = pd.DataFrame(columns = ['calibX', 'calibY', 'time'],index=range(25334))
        # Q,n,d,_ = self.fit_ellipse(a["magX"],a["magY"],a["magZ"])
        # Qinv = np.linalg.inv(Q)
        # b = -np.dot(Qinv, n)
        # Ainv = np.real(1 / np.sqrt(np.dot(n.T, np.dot(Qinv, n)) - d) * linalg.sqrtm(Q))
        # # A = Ainv.inv()
        # print(a["magX"].shape[0])
        # x = []
        # y = []
        for i in range(a["mag_field.x"].shape[0]):
            xvec = [a["mag_field.x"][i],a["mag_field.y"][i]]
            # print("xvec",np.shape(xvec))
            rot_data = np.matmul(pre_rot,xvec)
            mag["calibX"][i] = rot_data[0]/(ap/bp)
            mag["calibY"][i] = rot_data[1]
            # mag["calibZ"][i] = hHat[2]
            # print()
        # print(mag["calibY"])
        # mag['calibX'] = pd.Series(x)
        # mag['calibY'] = pd.Series(y)
        for i in range(a["mag_field.x"].shape[0]):
            xvec = [mag["calibX"][i],mag["calibY"][i]]
            # print("xvec",xvec.shape)
            rot_data = np.matmul(post_rot,xvec)
            nmag["calibX"][i] = rot_data[0]
            nmag["calibY"][i] = rot_data[1]

        nmag["time"] = a["header.stamp.secs"] - a["header.stamp.secs"][0]
        nmag.to_csv('calib_mag_all.csv')
        return mag["calibX"],mag["calibY"]



    
if __name__ == '__main__':
    bag_file_path = '/home/yash/Desktop/Lab_4/EECE5554/LAB4/src/Data/data_driving.bag'
    mag_csv = '/home/yash/Desktop/Lab_4/EECE5554/LAB4/src/Data/just_mag_data.csv'
    calibrated_mag_csv = '/home/yash/Desktop/Lab_4/EECE5554/LAB4/src/Data/calib_mag.csv'
    # bag_file_new_path = '/home/yash/Desktop/my_lab2/EECE5554/LAB3/src/Data/stat_new.bag'
    # magCal = magCal(csv_file_path)
    magCal = magCal(mag_csv)
    # magCal = magCal(mag_csv)
    # magCal.mag_data()
    mx,my,mz = magCal.remove_offset()
    u = magCal.fit_ellipse(mx,my)
    # print(u)
    # print(magCal.get_ell_pts(u))
    # x0, y0, ap, bp, e, phi = magCal.get_ell_pts(u)
    X,Y = magCal.calibrate_mag(u)
    Xa,Ya = magCal.calibrateWholeData(u)
    # u1 = magCal.fit_ellipse(X,Y)
    # x0, y0, ap, bp, e, phi = magCal.get_ell_pts(u1)
    # print(ap/bp)
    # calibratedX = np.array()
    # magCal.mag_data()
    # print(mx,my,mz)
    # magCal.mag_data()
