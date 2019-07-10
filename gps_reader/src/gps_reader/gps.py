#!/usr/bin/env python
import numpy as np
import math
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from gps_reader.msg import GPS_data

origLat = 0.0
origLon = 0.0
x_vel = 0.0
y_vel = 0.0
ang_vel = 0.0
gps_message = GPS_data()
pub_gps = rospy.Publisher('/GPS_coord', GPS_data, queue_size=1)

'''Reads the position in latitude and longitude. Converts it to x,y.
    Publishes custom message with positions: x,y,lon,lat and velocities: lin,ang'''
def GPS_posCallb(msg):
    global origLat, origLon, gps_message, pub_gps
    EARTH_RADIUS = 6371000;
    lat = msg.latitude
    lon = msg.longitude
    if origLat == 0.0 and origLon == 0.0:
        origLat = lat
        origLon = lon
    x = (lon - origLon) * np.pi/180 * np.cos((lat + origLat)/2 * np.pi/180) * EARTH_RADIUS
    y = (lat - origLat) * np.pi/180 * EARTH_RADIUS

    gps_message.x = x
    gps_message.y = y
    gps_message.latitude = lat
    gps_message.longitude = lon
    gps_message.x_vel = x_vel
    gps_message.y_vel = y_vel
    gps_message.ang_course = angleDiff(math.atan2(y_vel, x_vel))
    gps_message.ang_vel = ang_vel
    pub_gps.publish(gps_message)

'''Reads the linear and angular velocities'''
def GPS_velCallb(msg):
    global lin_vel, ang_vel
    x_vel = msg.twist.linear.x
    y_vel = msg.twist.linear.y
    ang_vel = msg.twist.angular.x

def angleDiff(angle):
    while angle > math.pi:
        angle = angle - 2 * math.pi
    while angle < -math.pi:
        angle = angle + 2 * math.pi

    return angle

def main():
    rospy.init_node('GPS_reader', anonymous=True)
    rospy.Subscriber("/fix", NavSatFix, GPS_posCallb)
    rospy.Subscriber("/vel", TwistStamped, GPS_velCallb)
    rospy.spin()
