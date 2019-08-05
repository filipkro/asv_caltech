#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import math
import numpy as np

left = false

def lidar_callb(msg):
    global left
    dist_th = rospy.get_param('dist_th')
    min_ang = msg.angle_min_
    ranges = msg.ranges
    inc = 2*math.pi/len(ranges)

    dists = []
    if left:
        indxs = np.arange(int(len(ranges)/6), int(len(ranges)/2), 1)
    else:
        indxs = np.arange(int(len(ranges)/2), int(5*len(ranges)/6), 1)

    dists = ranges[indxs]
    closer = np.nonzero(dists < dist_th)
    if len(closer) > 10:
        #TURN!!

def main():
    rospy.init_node('planner')
    rospy.Subscriber('os1/scan', LaserScan, lidar_callb)
