#!/usr/bin/env python
import roslib
import rospy
import time
import math
import sys
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

from tf.transformations import euler_from_quaternion


yaw_publisher        = rospy.Publisher("/yaw_imu", Float32, queue_size="1")
def imu_callback(msg):
    quat = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
    yaw = euler_from_quaternion(quat)[2] #yaw
    yaw_publisher.publish(yaw)

        

def main():
    try:
        rospy.init_node('yaw_imu_node')
        imu_topic = "/imu/data"
        imu_sub = rospy.Subscriber(imu_topic, Imu, imu_callback)  
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    # rospy.logware("Exiting")

if __name__ == "__main__":
    main()