#!/usr/bin/env python2
# -*- coding: UTF8 -*-
# PYTHON_ARGCOMPLETE_OK

import os
import numpy as np 
import pandas as pd
import sys 

import rospy
import rosbag
from sensor_msgs.msg import Imu, NavSatFix
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import math


INPUTS_PATH = "/media/phoenix/d2c96e98-3834-45b5-93dc-d99a3d923130/downloads/01_ph_ds/ros-ds/inputs"
GT_PATH = "/media/phoenix/d2c96e98-3834-45b5-93dc-d99a3d923130/downloads/01_ph_ds/ros-ds/gnd_truth"
RESULTS_PATH = "/media/phoenix/d2c96e98-3834-45b5-93dc-d99a3d923130/downloads/01_ph_ds/ros-ds/results"

def write_imu_gps_bag(dirpath, dirnames, filenames):
    with rosbag.Bag(dirpath + '/input.bag', 'w') as bag:
        imu_msg = Imu()
        gps_msg = NavSatFix()
        imu_flag = 0 ## 2 means imu msg gets compeleted.
        gps_flag = 0 ## 2 means gps msg gets compeleted.
        for f in filenames:
            if imu_flag == 2:
                imu_msg = Imu()
            if ".csv" in f:                        
                if "IMU" in f and "~" not in f:
                    ## IMU Topic                             
                    print("#####\nIMU IF \n##########\n")
                    if "_track" in f:
                        imu_flag+=1
                        print("#####\nIMU-Track IF \n##########\n")
                        imu_df = pd.read_csv(dirpath+'/'+f) 
                    else:
                        print("#####\nIMUx IF \n##########\n")
                        imu_raw_df = pd.read_csv(dirpath+'/'+f)
                        imu_flag+=1

                    if imu_flag == 2:  
                        for row in range(imu_df.shape[0]):                                    
                            timestamp = rospy.Time.from_sec(float(imu_df['GPSTime(sow)'][row]))                                    
                            imu_msg.header.frame_id = "imu_link"
                            imu_msg.header.stamp = timestamp                                
                            quaternion = quaternion_from_euler(imu_df['Roll(rad)'][row], imu_df['Pitch(rad)'][row], imu_df['Yaw(rad)'][row])

                            # Populate the data elements for IMU
                            imu_msg.orientation.x = quaternion[0]
                            imu_msg.orientation.y = quaternion[1]
                            imu_msg.orientation.z = quaternion[2]
                            imu_msg.orientation.w = quaternion[3]
                            
                            imu_msg.angular_velocity.x = imu_df['GyroX(rad/s)'][row]
                            imu_msg.angular_velocity.y = imu_df['GyroY(rad/s)'][row]
                            imu_msg.angular_velocity.z = imu_df['GyroZ(rad/s)'][row]

                            imu_msg.linear_acceleration.x = imu_raw_df['Accel_X (m/s2)'][row]
                            imu_msg.linear_acceleration.y = imu_raw_df['Accel_Y (m/s2)'][row]
                            imu_msg.linear_acceleration.z = imu_raw_df['Accel_Z(m/s2)'][row]                                                                                                                                                        

                            bag.write("/imu/data", imu_msg, t=timestamp)

                elif "GPS" in f and "~" not in f:
                    if "VRS-"  in f:
                        gps_df = pd.read_csv(dirpath+'/'+f) 
                        gps_flag +=1
                    else:
                        gps_flag+=1
                        gps_cov_df = pd.read_csv(dirpath+'/'+f) 

                    print("#####\nGPS IF \n##########\n")
                    
                    if gps_flag == 2:                           
                        for row in range(gps_df.shape[0]):                                    
                            timestamp = rospy.Time.from_sec(float(gps_df['GPSTime (sec)'][row]))  
                            # print(type(timestamp))                                                          
                            gps_msg.header.frame_id = "gps_link"
                            gps_msg.header.stamp = timestamp 
                            gps_msg.latitude = gps_df['Latitude (deg)'][row]
                            gps_msg.longitude = gps_df['Longitude (deg)'][row]
                            gps_msg.altitude = gps_df['Ellipsoidal_Height (m)'][row]
                            gps_msg.position_covariance_type = 2
                            gps_msg.position_covariance[0] = gps_cov_df['position_covariance_x (m^2)'][row]
                            gps_msg.position_covariance[4] = gps_cov_df['position_covariance_y (m^2)'][row]
                            gps_msg.position_covariance[8] = gps_cov_df['position_covariance_z (m^2)'][row]
                        
                            bag.write("/gps", gps_msg, t=timestamp)

def convert_gps2coordinates(lat, lon, alt):
    pass
def write_gnd_truth():
    pass
    
def main():
    for (dirpath, dirnames, filenames) in os.walk(INPUTS_PATH):
        if dirpath != INPUTS_PATH:
            write_imu_gps(dirpath, dirnames, filenames)
            print("#####\nFinished" + dirpath +  "\n##########\n")

                        
                    

if __name__ == '__main__':
    main()