from re import A
from Reader import Reader
from Math import GHKFilter
from utils import unify_measures
from math import sin, cos, radians

from matplotlib import pyplot as plt


if __name__ == "__main__":
    path = 'data/'
    gnss_data = "gnss_2022-07-25T11-26-49.032515.txt"
    imu_data = "lidar_1658737644300159744_IMU.log.csv"

    reader = Reader(path + gnss_data, path + imu_data)
    dt = reader.imu_period

    # init 
    gnss, imu = reader.current_pose()

    x_true = []
    y_true = []

    x_calc, y_calc = [], []

    try: 
        while True:
            if gnss != None:
                gnss_predict = unify_measures(gnss)

                if (len(x_calc) + len(y_calc)) == 0:
                    x_calc.append(gnss_predict['coords'][0])
                    y_calc.append(gnss_predict['coords'][1])

                x_true.append(gnss_predict['coords'][0])
                y_true.append(gnss_predict['coords'][1])

            else:
                x_calc.append(x_calc[-1] + gnss_predict['velocity']*sin(radians(gnss_predict['heading']))*dt)
                y_calc.append(y_calc[-1] + gnss_predict['velocity']*cos(radians(gnss_predict['heading']))*dt)

            gnss, imu = reader.next()

            # IMU BLOCK


    except StopIteration:
        plt.plot(x_true, y_true)
        plt.title('True coords')

        plt.plot(x_calc, y_calc)
        plt.title('Calc coords (V, Head)')

        plt.show()