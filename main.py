from Reader import Reader

if __name__ == "__main__":
    path = 'data/'
    gnss_data = "gnss_2022-07-25T11-26-49.032515.txt"
    imu_data = "lidar_1658737644300159744_IMU.log.csv"

    reader = Reader(path + gnss_data, path + imu_data)
    while True:
        a = reader.next()
        print(a)
        input()