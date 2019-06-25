#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, TwistStamped
from gps_reader.msg import GPS_data

origLat = 0.0
origLon = 0.0
lat = 0.0
lon = 0.0
x = 0.0
y = 0.0
lin_vel = 0.0
ang_vel = 0.0
#pose = PoseStamped()
gps_message = GPS_data()
#pub_poseStamped = rospy.Publisher('/GPS_coord', PoseStamped, queue_size=1)
pub_gps = rospy.Publisher('/GPS_coord', GPS_data, queue_size=1)


def GPS_posCallb(msg):
    global lat, lon, origLat, origLon, x, y, pose, pub_poseStamped, gps_message, pub_gps
    EARTH_RADIUS = 6371000;
    lat = msg.latitude
    lon = msg.longitude
    if origLat == 0.0 and origLon == 0.0:
        origLat = lat
        origLon = lon
    x = (lon - origLon) * np.pi/180 * np.cos((lat + origLat)/2 * np.pi/180) * EARTH_RADIUS
    y = (lat - origLat) * np.pi/180 * EARTH_RADIUS

    # pose.header.frame_id = "map"
    # pose.pose.position.x = x
    # pose.pose.position.y = y
    # pub_poseStamped.publish(pose)

    gps_message.x = x
    gps_message.y = y
    gps_message.latitude = lat
    gps_message.longitude = lon
    gps_message.lin_vel = lin_vel
    gps_message.ang_vel = ang_vel
    pub_gps.publish(gps_message)

def GPS_velCallb(msg):
    global lin_vel, ang_vel
    #is this the right data?
    lin_vel = msg.twist.linear.x
    ang_vel = msg.twist.angular.x

def main():
    rospy.init_node('GPS_reader', anonymous=True)
    # If PoseStamped is used for position, use either PoseStamped or gps
        # If GPS_data is used (i.e. all GPS data in the same message)
        #what topic? is that how it is written?
    rospy.Subscriber("/fix", NavSatFix, GPS_posCallb)
    rospy.Subscriber("/vel", TwistStamped, GPS_velCallb)
    rate = rospy.Rate(10)



    rospy.spin()

if __name__ == '__main__':
    main()
