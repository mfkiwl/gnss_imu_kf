from plot import Dead_reckoning
import os

path = 'data/'
gnss_data = path + "gnss_2022-07-25T11-26-49.032515.txt"
imu_data = []
for i in [i for i in os.listdir(path) if i.endswith('IMU.log.csv')]:
    imu_data.append(path + i)

if __name__ == "__main__":
    Dead_reckoning(gnss_data, imu_data)
    
