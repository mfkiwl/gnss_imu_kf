from Reader import Reader
from Math import GHKFilter
from utils import unify_measures

def ghk_filter_run(gnss_data, imu_data):
    reader = Reader(gnss_data, imu_data)

    dt = reader.imu_period
    g = 9.80665

    # init 
    gnss, imu = reader.current_pose()
    try: 
        while True:
            if gnss != None:
                gnss_predict = unify_measures(gnss)

    except StopIteration:
        print('End')
        exit()