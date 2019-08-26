#!/usr/bin/env python
import numpy as np
import math
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from gps_reader.msg import GPS_data

"""GPS redaer
This module contains the implementation details for the GPS_node.py
This is used with the nmea_navsat_driver package, which is a package that 
reads nmea data and parses them into lat and lon. This module subscribes to
the /fix publish by that nmea_navsat_driver. Note that we're not using /vel,
since it is for some reason returning the wrong value.


Attributes:
    origLat (float): origin reference point latitude
    origLon (float): origin reference point longitude
    gps_message (GPS_data): custom defined gps_data message
        see msg/GPS_data.msg
    pub_gps (rospy.Publisher): for publishing GPS_data to the 
        /GPS_coord topic. Remapped in one of the launch file
    x_prev (float): previous converetd x, y coordinate, for use
        in differentiating gps
    y_prev (float): same as x_prev
    t_prev (rospy.Time): previous time, for differentiating
    angC_prev (float): previous angular velocity

Example:
    this node should be ran with the nmea driver. Don't forget to connect the 
    GPS to the jetson!! If you encounter permission issue, run:

        sudo chmod 666 /dev/serial/by-id/*

    rosrun commands:
        rosrun nmea_navsat_driver nmea_serial_driver _port:="/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0" _baud:=119200
        rosrun gps_reader GPS_node.py
    

"""

origLat = 0.0
origLon = 0.0
gps_message = GPS_data()
pub_gps = rospy.Publisher('/GPS_coord', GPS_data, queue_size=10)
x_prev = 0.0
y_prev = 0.0
t_prev = rospy.Time()
angC_prev = 0.0


def GPS_posCallb(msg):
    '''Call back function on the /fix published by the nmea driver
    Reads the position in latitude and longitude. Converts it to x,y.
    Publishes custom message with positions: x,y,lon,lat and velocities: lin,ang
    
    Args: 
        msg (NavSatFix) message published by the /fix'''
    global origLat, origLon, gps_message, pub_gps, x_vel, y_vel, ang_vel

    EARTH_RADIUS = 6371000

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

def GPS_velCallb(msg):
    '''Reads the linear and angular velocities. Not used.'''
    global x_vel, y_vel, ang_vel
    x_vel = msg.twist.linear.x
    y_vel = msg.twist.linear.y
    ang_vel = msg.twist.angular.z

def angleDiff(angle):
    '''Bound an angle between -pi and +pi in radians
    
    Args:
        angle (float): angle to be bounded
    Returns:
        float: angle that is bounded
    '''
    while angle > math.pi:
        angle = angle - 2 * math.pi
    while angle < -math.pi:
        angle = angle + 2 * math.pi

    return angle

def getVel(x,y,t):
    ''' Differentiate position and angle postiion for velocity

    Args:
        x (float): current x position
        y (float): current y position
        t (rospy.Time): current time
    
    Returns:
        x_vel (float): diferentiated x velocity
        y_vel (float):diferentiated x velocity
        ang_course (float): direction of travel 
        angC_prev (float): angular velocity for the velocity vector

    '''

    global x_prev, y_prev, t_prev, angC_prev
    timeDiff = (t-t_prev).to_sec()
    x_vel = (x - x_prev)/timeDiff
    y_vel = (y - y_prev)/timeDiff
    ang_course = angleDiff(math.atan2(y_vel, x_vel))
    ang_vel = (ang_course - angC_prev)/timeDiff
    x_prev = x
    y_prev = y
    t_prev = t

    return x_vel, y_vel, ang_course, angC_prev

def main():
    rospy.init_node('GPS_reader', anonymous=True)
    rospy.Subscriber("/fix", NavSatFix, GPS_posCallb)
    rospy.Subscriber("/vel", TwistStamped, GPS_velCallb)
    rospy.spin()
