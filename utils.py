def hhmmss(synced_time):
    ls = list(map(lambda x: x.strftime('%H:%M:%S.%f'), synced_time))
    print("GNSS time: ", ls[0])
    print("IMU time: ", ls[1], '\n')
    return synced_time

def convertBin2CSV(filename):
    import openpylivox as opl
    filename = 'lidar_1658737644300159744.log'
    opl.convertBin2CSV(filename)

def pos_dict():
    pose = {"GNSS": {"timestamp": None, 
                    "coords": None, 
                    "heading": None, 
                    "velocity": None,
                    "accuracy": None},
            "IMU": {"gyro_x": None,
                    "gyro_y": None,
                    "gyro_z": None,
                    "acc_x": None,
                    "acc_y": None,
                    "acc_z": None,
                    "time": None}} 
    return pose