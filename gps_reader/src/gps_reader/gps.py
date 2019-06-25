import numpy as np
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, Twist
from gps_reader.msg import GPS_data

origLat = 0.0
origLon = 0.0
lat = 0.0
lon = 0.0
x = 0.0
y = 0.0
lin_vel = 0.0
ang_vel = 0.0

def GPS_posCallb(msg):
    EARTH_RADIUS = 6371000;
    lat = msg.latitude
    lon = msg.longitude
    x = (lon - origLon) * np.pi/180 * np.cos((lat + origLat)/2 * np.pi/180) * EARTH_RADIUS
    y = (lat - origLat) * np.pi/180 * EARTH_RADIUS

def GPS_velCallb(msg):
    #is this the right data?
    lin_vel = msg.linear.x
    ang_vel = msg.angular.x

def main():
    rospy.init_node('GPS_reader', anonymous=True)
    # If PoseStamped is used for position, use either PoseStamped or gps
    pub_poseStamped = rospy.Publisher('/GPS_coord', PoseStamped, queue_size=1)
    # If GPS_data is used (i.e. all GPS data in the same message)
    pub_gps = rospy.Publisher('/GPS_coord', GPS_data, queue_size=1)
    #what topic? is that how it is written?
    rospy.Subscriber("/fix", NavSatFix, GPS_posCallb)
    rospy.Subscriber("/vel", Twist, GPS_velCallb)
    rate = rospy.Rate(10)
    rate.sleep()
    origLat = lat
    origLon = lon
    pose = PoseStamped()
    gps_message = GPS_data()

    while not rospy.is_shutdown():
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y

        gps_message.x = x
        gps_message.y = y
        gps_message.latitude = lat
        gps_message.longitude = lon
        gps_message.lin_vel = lin_vel
        gps_message.ang_vel = ang_vel

        pub_poseStamped.publish(pose)
        pub_gps.publish(gps_message)

if __name__ == '__main__':
    main()
