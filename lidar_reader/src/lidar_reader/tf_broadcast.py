#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from lidar_reader.msg import lidar_data
from gps_reader.msg import GPS_data
from std_msgs.msg import Float32, Float32MultiArray
from motor_control.msg import Arduino_imu
import tf, math

br = tf.TransformBroadcaster()
current_Euler = [0.0,0.0,0.0]
current_xy = [0.0,0.0]

# only for testing purpose, should be removed during live run
origin = [0.0, 0.0]
first_run = True



def GPS_callb(msg):
    global current_xy, current_Euler, br, first_run, origin

    #if first_run:
    #    origin = [msg.x, msg.y]
    #    first_run = False
    current_xy[0] = msg.x #- origin[0]
    current_xy[1] = msg.y #- origin[1]
    
    br.sendTransform((current_xy[0], current_xy[1], 0),
                     tf.transformations.quaternion_from_euler(current_Euler[0], current_Euler[1], current_Euler[2]),
                     rospy.Time.now(),
                     "os1_lidar",
                     "map")
    #print(current_xy)
    

def IMU_callb(msg):
    global current_xy, current_Euler, br
    imu_data = msg.data
    #imu_offset = 109.0/180 * math.pi
    current_Euler = [0.0, 0.0, imu_data]
    br.sendTransform((current_xy[0], current_xy[1], 0),
                     tf.transformations.quaternion_from_euler(0, 0, imu_data),
                     rospy.Time.now(),
                     "os1_lidar",
                     "map")

def main():
    rospy.init_node('lidar_reader', anonymous=True)
    rospy.Subscriber("/GPS/xy_coord", GPS_data, GPS_callb)
    rospy.Subscriber('/heading', Float32, IMU_callb)
    rospy.spin()

if __name__ == '__main__':
    main()
