#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from lidar_reader.msg import lidar_data
import math.pi as pi

'''Lidar
This module is unused in the current code. But please add lidar related script to here
if lidar data were to be polished
'''

pub_lidar = rospy.Publisher('/laserSimp', GPS_data, queue_size=1)
lidar_msg = lidar_data()

def laserCallb(msg):
    global pub_lidar, lidar_msg
    inc = msg.angle_increment
    min = msg.angle_min
    i_left = -(pi/2 + min) / inc
    i_right = (pi/2 - min) / inc
    left = msg.ranges[i_left]
    right = msg.ranges[i_right]

    lidar_msg.left = left
    lidar_msg.right = right
    pub_lidar.publish(lidar_msg)


def main():
    rospy.init_node('lidar_reader', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, laserCallb)
    rospy.spin()

if __name__ == '__main__':
    main()
