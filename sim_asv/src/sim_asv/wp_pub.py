#!/usr/bin/env python

import rospy
from gps_reader.msg import GPS_WayPoints, GPS_data

def main():
    rospy.init_node('wpCreator')
    point = GPS_data()
    wpList = GPS_WayPoints()
    pub = rospy.Publisher('ControlCenter/gps_wp', GPS_WayPoints, queue_size=10)
    list = []
    point.x = 2.0
    point.y = 0.0
    list.append(point)

    point.x = 5.0
    point.y = 2.0
    list.append(point)

    point.x = 7.0
    point.y = 6.0
    list.append(point)

    point.x = 5.0
    point.y = 5.0
    list.append(point)

    point.x = 3.0
    point.y = 2.0
    list.append(point)

    point.x = 0.0
    point.y = 0.0
    list.append(point)

    wpList = list
    pub.publish(wpList)



if __name__ == '__main__':
    main()
