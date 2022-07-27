from datetime import datetime, timedelta
import json
import csv
import re
from time import sleep

from utils import hhmmss, pos_dict

class Reader(object):
    def __init__(self, gnss_data, imu_data):
        self.gps_period = 1. #sec
        self.imu_period = 1./200 #sec
        self.__gnss = self.__read_gnss(gnss_data)
        self.__imu = self.__read_imu(imu_data)
        self.__pose = pos_dict()
        self.__time_sync()
    
    
    def current_pose(self):
        return self.__pose['GNSS'], self.__pose['IMU']

    def next(self):
        """ Updates GNSS/IMU pose from generators"""
        sleep(self.imu_period)
        if abs(self.__pose['GNSS']['timestamp'] - self.__pose['IMU']['time']) \
            < timedelta(seconds=1) - timedelta(milliseconds=1000*self.imu_period):
            imu_row, imu_time = next(self.__imu)
            self.__update_pose(None, None, imu_row, imu_time)
            return None, self.__pose['IMU'] 
        else:
            gnss_row, gnss_time = next(self.__gnss)
            imu_row, imu_time = next(self.__imu)
            self.__update_pose(gnss_row, gnss_time, imu_row, imu_time)
            return self.__pose['GNSS'], self.__pose['IMU']        
            

    def __read_gnss(self, gnss_data):
        """
        Yields current line and datetime (UTC+3) for GNSS data
        """
        file1 = open(gnss_data, 'r')
        while True:
            line = file1.readline()
            if not line:
                break
            if len(line) > 0:
                data = json.loads(line)
                dt = (datetime.fromtimestamp(data['timestamp']) + timedelta(hours=3))
                yield line, dt

    def __read_imu(self, imu_data):
        """
        1. Find start_time on IMU as regexp on csv name
        2. Adds row time deltas to start_time 
        3. Yields current row and current timestamp (UTC+3)
        """
        start_time = datetime.fromtimestamp(float(re.findall('[0-9]+', imu_data)[0])*(10**-9))
        with open(imu_data) as imu:
            csv_reader = csv.reader(imu)
            csv_reader.__next__() #skip header row
            last = float(next(csv_reader)[-1])
            for row in csv_reader:
                start_time += timedelta(seconds = (float(row[-1]) - last))
                last = float(row[-1])
                yield row, start_time

    def __time_sync(self):
        """
        1. Goes to first GNSS_second > IMU_second condition
        2. Takes closest millisecond on IMU (now for 200HZ)
        3. Updates GNSS/IMU pose and their cur timestamps 
        """
        gnss_row, gnss_time = next(self.__gnss)
        imu_row, imu_time = next(self.__imu)
        while self.__gnss and self.__imu:
            while gnss_time < imu_time:
                gnss_row, gnss_time = next(self.__gnss)
            while abs(gnss_time - imu_time) > timedelta(milliseconds=float(1000*self.imu_period/2)):
                imu_row, imu_time = next(self.__imu)
            break
        if self.__gnss and self.__imu:
            self.__update_pose(gnss_row, gnss_time, imu_row, imu_time)
        else:
            print('Out of sync.')
            exit()


    def __update_pose(self, gnss_row = None, gnss_time = None, imu_row = None, imu_time = None):
        """Update util"""
        if gnss_row != None and gnss_time != None:
            self.__pose['GNSS'].update((json.loads(gnss_row)))
            self.__pose['GNSS']['timestamp'] = gnss_time
        if imu_row != None and imu_time != None:
            for key, value in zip(self.__pose['IMU'].keys(), imu_row):
                self.__pose['IMU'][key] = value
            self.__pose['IMU']['time'] = imu_time

        
