from Reader import Reader
from Math import GHKFilter
from utils import unify_measures
from math import sin, cos, radians, pi, sqrt, atan2

from matplotlib import pyplot as plt


if __name__ == "__main__":
    path = 'data/'
    gnss_data = "gnss_2022-07-25T11-26-49.032515.txt"
    imu_data = "lidar_1658737644300159744_IMU.log.csv"

    reader = Reader(path + gnss_data, path + imu_data)
    dt = reader.imu_period
    g = 9.80665

    # init 
    gnss, imu = reader.current_pose()

    x_true, y_true = [], []
    x_nmea, y_nmea = [], []
    x_imu_pos, y_imu_pos = [], []

    head_true = []
    head_nmea = []
    head_imu = []

    vel_true = []
    vel_nmea = []
    vel_imu = []

    try: 
        while True:
            if gnss != None:
                gnss_predict = unify_measures(gnss)

                if (len(x_imu_pos) + len(y_imu_pos)) == 0:
                    x_nmea.append(gnss_predict['coords'][0])
                    y_nmea.append(gnss_predict['coords'][1])

                    x_imu_pos.append(gnss_predict['coords'][0])
                    y_imu_pos.append(gnss_predict['coords'][1])

                    vel_true.append(1.8*gnss_predict['velocity'])
                    vel_nmea.append(1.8*gnss_predict['velocity'])
                    vel_imu.append(1.8*gnss_predict['velocity'])

                    head_true.append(gnss_predict['heading'])
                    head_nmea.append(gnss_predict['heading'])
                    head_imu.append(gnss_predict['heading'])


                x_true.append(gnss_predict['coords'][0])
                y_true.append(gnss_predict['coords'][1])

            else:

                x_nmea.append(x_nmea[-1] + 1.8*gnss_predict['velocity']*sin(radians(gnss_predict['heading']))*dt)
                y_nmea.append(y_nmea[-1] + 1.8*gnss_predict['velocity']*cos(radians(gnss_predict['heading']))*dt)

                head_nmea.append(gnss_predict['heading'])
                head_imu.append(head_imu[-1] - float(imu['gyro_z'])*dt*180/pi)
                
                vel_nmea.append(1.8*gnss_predict['velocity'])
                vel_imu.append(vel_imu[-1] - float(imu['acc_x'])*g*dt)

                x_imu_pos.append(x_imu_pos[-1] + vel_imu[-1]*dt*sin(radians(head_imu[-1])))
                y_imu_pos.append(y_imu_pos[-1] + vel_imu[-1]*dt*cos(radians(head_imu[-1])))
                

            if len(x_true) + len(y_true) >= 4:
                head_true.append(atan2(x_true[-1]-x_true[-2], y_true[-1]-y_true[-2])*180/pi)
                vel_true.append(sqrt((x_true[-1] - x_true[-2])**2 + (y_true[-1] - y_true[-2])**2))
            else:
                head_true.append(gnss_predict['heading'])
                vel_true.append(1.8*gnss_predict['velocity'])
                
            gnss, imu = reader.next()


    except StopIteration:
        plt.subplot(3, 1, 1)
        plt.plot(x_true, y_true, marker="o", markersize=2, label = 'True GNSS')
        plt.plot(x_imu_pos, y_imu_pos, marker="v", markersize=1.5, label = 'True IMU')
        plt.plot(x_nmea, y_nmea, marker = '*', markersize=1.5, label = 'True NMEA')
        plt.legend()
        plt.title('pose')

        plt.subplot(3, 1, 2)
        plt.plot([i for i in range(len(vel_true))], vel_true, marker="o", markersize=2, label = 'GNSS speed')
        plt.plot([i for i in range(len(vel_imu))], vel_imu, marker="v", markersize=1.5, label = 'IMU speed')
        plt.plot([i for i in range(len(vel_nmea))], vel_nmea, marker = '*', markersize=1.5, label = 'NMEA speed')
        plt.legend()
        plt.title('velocity')

        plt.subplot(3, 1, 3)
        plt.plot([i for i in range(len(head_true))], head_true, marker="o", markersize=2, label = 'True heading')
        plt.plot([i for i in range(len(head_imu))], head_imu, marker="v", markersize=1.5, label = 'IMU heading')
        plt.plot([i for i in range(len(head_nmea))], head_nmea, marker = '*', markersize=1.5, label = 'NMEA heading')
        plt.legend()
        plt.title('Heading')

        plt.show()