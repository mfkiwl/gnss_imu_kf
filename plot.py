from Reader import Reader
from utils import unify_measures, Grid_to_WGS84_iter
from math import sin, cos, radians, pi, sqrt, atan2
from matplotlib import pyplot as plt
from Filter import GHFilter


def Dead_reckoning(gnss_data, imu_data):
    reader = Reader(gnss_data, imu_data)
    dt = reader.imu_period
    g = 9.80665

    # init 
    gnss, imu = reader.current_pose()

    pos_true = []
    pos_nmea = []
    pos_imu = []
    pos_ghf = []

    head_true = []
    head_nmea = []
    head_imu = []
    head_ghf = []

    vel_true = []
    vel_nmea = []
    vel_imu = []
    vel_ghf = []

    try: 
        while True:
            if gnss != None:
                gnss_predict = unify_measures(gnss)
                pos_true.append(gnss_predict['coords'])

                if len(pos_imu) == 0:
                    pos_nmea.append(gnss_predict['coords'])
                    pos_imu.append(gnss_predict['coords'])
                    pos_ghf.append(gnss_predict['coords'])

                    vel_true.append(1.8*gnss_predict['velocity'])
                    vel_nmea.append(1.8*gnss_predict['velocity'])
                    vel_imu.append(1.8*gnss_predict['velocity'])

                    head_true.append(gnss_predict['heading'])
                    head_nmea.append(gnss_predict['heading'])
                    head_imu.append(gnss_predict['heading'])

                    ghfilter = GHFilter(x = gnss_predict['coords'], \
                                        dx = [1.8*gnss_predict['velocity']*sin(radians(gnss_predict['heading']))*dt, \
                                              1.8*gnss_predict['velocity']*cos(radians(gnss_predict['heading']))*dt], \
                                        dt = dt, g = 1/(gnss_predict['accuracy'][0]), h = 1/(gnss_predict['accuracy'][0])) 

            pos_nmea.append((pos_nmea[-1][0] + 1.8*gnss_predict['velocity']*sin(radians(gnss_predict['heading']))*dt, \
                            pos_nmea[-1][1] + 1.8*gnss_predict['velocity']*cos(radians(gnss_predict['heading']))*dt))
            
            vel_nmea.append(1.8*gnss_predict['velocity'])
            head_nmea.append(gnss_predict['heading'])

            head_imu.append(head_imu[-1] - float(imu['gyro_z'])*dt*180/pi)
            vel_imu.append(vel_imu[-1] - float(imu['acc_x'])*g*dt)
            pos_imu.append((pos_imu[-1][0] + vel_imu[-1]*dt*sin(radians(head_imu[-1])), \
                            pos_imu[-1][1] + vel_imu[-1]*dt*cos(radians(head_imu[-1]))))

            #GH-Filter Update step
            gh_dx = [vel_imu[-1]*dt*sin(radians(head_imu[-1])), \
                           vel_imu[-1]*dt*cos(radians(head_imu[-1]))]
            gh_x = pos_imu[-1]
            f_point, f_v = ghfilter.update(z = pos_nmea[-1], dx_pred = gh_dx, x_pred = gh_x)

            pos_ghf.append((f_point[0], f_point[1]))           

            print(sqrt((pos_nmea[-1][0] - pos_ghf[-1][0])**2 + (pos_nmea[-1][1] - pos_ghf[-1][1])**2))
            
            if len(pos_true) >= 2:
                head_true.append(atan2(pos_true[-1][0]-pos_true[-2][0], pos_true[-1][1]-pos_true[-2][1])*180/pi)
                vel_true.append(sqrt((pos_true[-1][0] - pos_true[-2][0])**2 + (pos_true[-1][1] - pos_true[-2][1])**2))

                head_ghf.append(atan2(pos_ghf[-1][0]-pos_ghf[-2][0], pos_ghf[-1][1]-pos_ghf[-2][1])*180/pi)
                vel_ghf.append(sqrt((pos_ghf[-1][0] - pos_ghf[-2][0])**2 + (pos_ghf[-1][1] - pos_ghf[-2][1])**2)/dt)
            else:
                head_true.append(gnss_predict['heading'])
                vel_true.append(1.8*gnss_predict['velocity'])

                head_ghf.append(gnss_predict['heading'])

            gnss, imu = reader.next()
        

    except StopIteration:
        pos_true_WGS84 = [pt for pt in Grid_to_WGS84_iter(pos_true)]
        pos_imu_WGS84 = [pt for pt in Grid_to_WGS84_iter(pos_imu)]
        pos_nmea_WGS84 = [pt for pt in Grid_to_WGS84_iter(pos_nmea)]
        pos_ghf_WGS84 = [pt for pt in Grid_to_WGS84_iter(pos_ghf)]

        plt.subplot(3, 1, 1)
        plt.plot([i[1] for i in pos_true_WGS84], [i[0] for i in pos_true_WGS84], marker="o", markersize=2, label = 'True GNSS')
        plt.plot([i[1] for i in pos_imu_WGS84], [i[0] for i in pos_imu_WGS84], marker="v", markersize=1.5, label = 'True IMU')
        plt.plot([i[1] for i in pos_nmea_WGS84], [i[0] for i in pos_nmea_WGS84], marker = '*', markersize=1.5, label = 'True NMEA')
        plt.plot([i[1] for i in pos_ghf_WGS84], [i[0] for i in pos_ghf_WGS84], marker = '>', markersize=2, label = 'GH-Filter')
        plt.legend()
        plt.title('pose')

        plt.subplot(3, 1, 2)
        plt.plot([i for i in range(len(vel_true))], vel_true, marker="o", markersize=2, label = 'GNSS speed')
        plt.plot([i for i in range(len(vel_imu))], vel_imu, marker="v", markersize=1.5, label = 'IMU speed')
        plt.plot([i for i in range(len(vel_nmea))], vel_nmea, marker = '*', markersize=1.5, label = 'NMEA speed')
        plt.plot([i for i in range(len(vel_ghf))], vel_ghf, marker = '>', markersize=1.5, label = 'GHF speed')
        plt.legend()
        plt.title('velocity')

        plt.subplot(3, 1, 3)
        plt.plot([i for i in range(len(head_true))], head_true, marker="o", markersize=2, label = 'True heading')
        plt.plot([i for i in range(len(head_imu))], head_imu, marker="v", markersize=1.5, label = 'IMU heading')
        plt.plot([i for i in range(len(head_nmea))], head_nmea, marker = '*', markersize=1.5, label = 'NMEA heading')
        plt.plot([i for i in range(len(head_ghf))], head_ghf, marker = '>', markersize=1.5, label = 'GHF heading')
        plt.legend()
        plt.title('Heading')

        plt.show()