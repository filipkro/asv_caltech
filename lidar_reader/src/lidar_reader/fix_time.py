#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu, PointCloud2

pub_PC = rospy.Publisher('os1/lidar', PointCloud2, queue_size=10)
pub_imu = rospy.Publisher('os1/imu', Imu, queue_size=10)

def pc_callb(msg):
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = 'os1_lidar'
    pub_PC.publish(msg)

def imu_callb(msg):
    msg.header.stamp = rospy.get_rostime()
    pub_imu.publish(msg)

def main():
    rospy.init_node('fix_timestamps')
    rospy.Subscriber('os1_cloud_node/points', PointCloud2, pc_callb)
    rospy.Subscriber('os1_cloud_node/imu', Imu, imu_callb)

    rospy.spin()

if __name__ == '__main__':
    main()
