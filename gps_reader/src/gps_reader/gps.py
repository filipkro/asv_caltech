#!/usr/bin/env python
import numpy as np
import math
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from gps_reader.msg import GPS_data

origLat = 0.0
origLon = 0.0
gps_message = GPS_data()
pub_gps = rospy.Publisher('/GPS_coord', GPS_data, queue_size=10)
x_prev = 0.0
y_prev = 0.0
t_prev = 0.0
angC_prev = 0.0

'''Reads the position in latitude and longitude. Converts it to x,y.
    Publishes custom message with positions: x,y,lon,lat and velocities: lin,ang'''
def GPS_posCallb(msg):
    global origLat, origLon, gps_message, pub_gps, x_vel, y_vel, ang_vel

    EARTH_RADIUS = 6371000;

    origLat = float(rospy.get_param('/originLat', 0.0))
    origLon = float(rospy.get_param('/originLon', 0.0))

    lat = msg.latitude
    lon = msg.longitude

    x = (lon - origLon) * np.pi/180 * np.cos((lat + origLat)/2 * np.pi/180) * EARTH_RADIUS
    y = (lat - origLat) * np.pi/180 * EARTH_RADIUS

    x_vel, y_vel, ang_course, ang_vel = getVel(x, y, msg.header.stamp)

    gps_message.x = x
    gps_message.y = y
    gps_message.latitude = lat
    gps_message.longitude = lon
    gps_message.origin_lat = origLat
    gps_message.origin_lon = origLon
    gps_message.x_vel = x_vel
    gps_message.y_vel = y_vel
    gps_message.ang_course = ang_course
    gps_message.ang_vel = ang_vel


    pub_gps.publish(gps_message)

'''Reads the linear and angular velocities'''

def GPS_velCallb(msg):
    global x_vel, y_vel, ang_vel
    x_vel = msg.twist.linear.x
    y_vel = msg.twist.linear.y
    ang_vel = msg.twist.angular.z

def angleDiff(angle):
    while angle > math.pi:
        angle = angle - 2 * math.pi
    while angle < -math.pi:
        angle = angle + 2 * math.pi

    return angle

def getVel(x,y,t):
    global x_prev, y_prev, t_prev, angC_prev
    x_vel = (x - x_prev)/(t - t_prev)
    y_vel = (y - y_prev)/(t - t_prev)
    ang_course = angleDiff(math.atan2(y_vel, x_vel))
    ang_vel = (ang_course - angC_prev)/(t - t_prev)
    x_prev = x
    y_prev = y
    t_prev = t

    return x_vel, y_vel, ang_course, angC_prev

def main():
    rospy.init_node('GPS_reader', anonymous=True)
    rospy.Subscriber("/fix", NavSatFix, GPS_posCallb)
    rospy.Subscriber("/vel", TwistStamped, GPS_velCallb)
    rospy.spin()
