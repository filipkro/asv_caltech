#!/usr/bin/env python

import rospy
from gps_reader.msg import GPS_data
from visualization_msgs.msg import Marker, MarkerArray

pub = rospy.Publisher('vizMarkArray', MarkerArray, queue_size=1)
markerArray = MarkerArray()

def GPS_callb(msg):
    global pub

    marker = Marker()
    marker.header.frame_id = "/map"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.a = 1.0
    marker.color.r = 1.0 # make it red
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0

    marker.pose.position.x = msg.x
    marker.pose.position.y = msg.y
    marker.pose.position.z = 0

    markerArray.markers.append(marker)

    # Renumber the marker IDs
    id = 0
    for m in markerArray.markers:
        m.id = id
        id += 1

    pub.publish(markerArray)

def main():
    rospy.init_node('visualize')
    rospy.Subscriber('GPS/xy_coord', GPS_data, GPS_callb)

    rospy.spin()
