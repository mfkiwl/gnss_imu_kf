from re import A
from Reader import Reader
from Math import GHKFilter
from utils import unify_measures
import numpy as np


if __name__ == "__main__":
    path = 'data/'
    gnss_data = "gnss_2022-07-25T11-26-49.032515.txt"
    imu_data = "lidar_1658737644300159744_IMU.log.csv"

    reader = Reader(path + gnss_data, path + imu_data)
    dt = reader.imu_period

    # init 
    gnss, imu = reader.current_pose()

    while True:
        if gnss != None:
            gnss_predict = unify_measures(gnss)
        else:
            gnss_predict['coords'][0] += gnss_predict["velocity"]*np.cos(gnss_predict["heading"])*dt 
            gnss_predict['coords'][1] += gnss_predict["velocity"]*np.sin(gnss_predict["heading"])*dt
        gnss, imu = reader.next()
        print(gnss_predict)
        print(imu)

        input()