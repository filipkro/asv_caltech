#!/usr/bin/env python

import rospy
from gps_reader.msg import GPS_data
from motor_control.msg import MotorCommand
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Float32, Float32MultiArray
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
import math
import numpy as np
import tf

pub_gps = rospy.Publisher('/GPS/xy_coord', GPS_data, queue_size=10)
pub_imu = rospy.Publisher('/heading', Float32, queue_size=10)
pub_adcp = rospy.Publisher('adcp/data', Float32MultiArray, queue_size=10)
pub_odom = rospy.Publisher('sim/pose', Odometry, queue_size=10)
pub_lidar = rospy.Publisher('os1/scan', LaserScan, queue_size=1)
walls = MarkerArray()

x = 0.0
y = 0.0

omega = 0.0
v = 0.0
theta = math.pi/2
gps_message = GPS_data()

lidar_seq = 0

def add_wall(x1,y1,x2,y2):
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD

    # marker scale
    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.3

    # marker color
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    # marker orientaiton
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # marker position
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0

    # marker line points
    marker.points = []
    # first point
    first_line_point = Point()
    first_line_point.x = x1
    first_line_point.y = y1
    first_line_point.z = 0.0
    marker.points.append(first_line_point)
    # second point
    second_line_point = Point()
    second_line_point.x = x2
    second_line_point.y = y2
    second_line_point.z = 0.0
    marker.points.append(second_line_point)

    walls.markers.append(marker)

def setup_walls():
    global walls
    pub_wall = rospy.Publisher('walls', MarkerArray, queue_size=1)
    add_wall(-10, -10, -10, 10)
    add_wall(10, -10, 10, 10)

    id = 0
    for m in walls.markers:
        m.id = id
        id += 1

    pub_wall.publish(walls)

def update_lidar():
    global x, theta, lidar_seq, pub_lidar
    # print("IN lidar_seq")
    lidar_len = 400
    scan = LaserScan()
    inc = 2*math.pi/lidar_len
    ranges = []
    for i in range(lidar_len):
        phi_lidar = -math.pi + i*inc
        phi = theta + phi_lidar
        if math.pi/2 - 0.1 <= abs(phi) <= math.pi/2 + 0.1:
            ranges.append(100)
        else:
            if abs(phi) < math.pi/2:
                dx = 10 - x
            else:
                dx = -10 - x
            r = dx/math.cos(phi)
            if 0 <= r < 40:
                ranges.append(r)
            else:
                ranges.append(100)


    scan.header.stamp = rospy.get_rostime()
    scan.header.frame_id = "os1_lidar"
    scan.header.seq = lidar_seq
    lidar_seq += 1
    scan.angle_min = -math.pi
    scan.angle_max = math.pi
    scan.angle_increment = inc
    scan.ranges = ranges

    pub_lidar.publish(scan)



def angleDiff(angle):
    while angle > math.pi:
        angle = angle - 2 * math.pi
    while angle < -math.pi:
        angle = angle + 2 * math.pi

    return angle


#def update_state(self, state, uR, uL, rudder):
def update_state(msg):
    ''' for simulation '''
    global pub_gps, gps_message, pub_imu, x, y, v, omega, theta, pub_odom

    #uR = -uR/ 100
    #uL = -uL/ 100

    update_lidar()


    uR = msg.strboard/100.0
    uL = msg.port/100.0

    rudder = msg.servo

    current_v = rospy.get_param('v_current', 0.5)
    current_ang = rospy.get_param('current_ang', math.pi/2)
    current = Float32MultiArray()
    current.data = [current_v, current_ang]

    # Rudder Angle
    rudder_rate = (1894.0 - 1195.0)/90.0
    rudder_ang = (rudder - 1600.0) / rudder_rate
    rudder_ang = -rudder_ang / 180.0 * math.pi

    b_l = 4.0 # sim linear drag
    b_r = 2.5 # sim rotational drag
    I_zz = 1.0 # sim moment of inertia
    m = 5.0 # sim mass
    robot_radius = 0.5
    x_cg = 0.9
    w = 20.0

    dt = 0.2

   # update state
    a = (uR + uL)/m - b_l/m * v
    # print(a)
    ang_acc = -b_r / I_zz * omega  +  (uR + uL) * math.sin(rudder_ang) * x_cg / I_zz

    v = v + a * dt
    omega = omega + ang_acc * dt

    robot_vx = v * math.cos(theta) - current_v * math.cos(current_ang)
    robot_vy = v * math.sin(theta) - current_v * math.sin(current_ang)
    v_robot = np.array([robot_vx, robot_vy])

    ang_course = math.atan2(v_robot[1], v_robot[0])
    v_course = math.sqrt(np.dot(v_robot,v_robot))

    omega = min(max(omega, -1.0), 1.0)

    # update position
    # x = x + v*math.cos(angleDiff(theta)) * dt + current_v * math.cos(angleDiff(current_ang)) * dt
    # y = y + v*math.sin(angleDiff(theta)) * dt + current_v * math.sin(angleDiff(current_ang))* dt
    x = x + robot_vx * dt
    y = y + robot_vy * dt
    theta = angleDiff(theta + omega * dt)

    gps_message.x = x
    gps_message.y = y
    gps_message.x_vel = robot_vx
    gps_message.y_vel = robot_vy
    gps_message.ang_course = ang_course

    # quat = qfe(0,0,theta)
    # pose.pose.position.x = x
    # pose.pose.position.y = y
    # pose.pose.position.z = 0.0
    # pose.pose.orientation.x = quat[0]
    # pose.pose.orientation.y = quat[1]
    # pose.pose.orientation.z = quat[2]
    # pose.pose.orientation.w = quat[3]
    # pose.header.frame_id = "/map"

    pub_gps.publish(gps_message)
    pub_imu.publish(theta)
    pub_adcp.publish(current)
    # pub_pose.publish(pose)

    # print("x: ", x)
    # print("y: ", y)
    # print("theta: ", theta)
    # print("xvel: ", robot_vx)
    # print("yvel: ", robot_vy)
    # print("ang_course: ", ang_course)
    # print("current_v", current_v)
    # print("current_ang", current_ang)

    # odom message

    odom_msg = Odometry()
    odom_msg.pose.pose.position.x = x
    odom_msg.pose.pose.position.y = y

    quat = tf.transformations.quaternion_from_euler(0, 0, theta)
    odom_msg.pose.pose.orientation.x = quat[0]
    odom_msg.pose.pose.orientation.y = quat[1]
    odom_msg.pose.pose.orientation.z = quat[2]
    odom_msg.pose.pose.orientation.w = quat[3]
    odom_msg.header.frame_id = 'map'
    pub_odom.publish(odom_msg)




    # print(state.v)
    # print(state.theta)
    print("x, y:", x, y)
    print('current: ', current.data[1])
    # print(state.y)
    # self.state_est.lat, self.state_est.lon = utm.to_latlon(self.utm_x, self.utm_y, 11, 'S')

    # keep this to return the updated state



def main():
    rospy.init_node('sim')
    rospy.Subscriber('motor_controller/motor_cmd_reciever', MotorCommand, update_state)
#    rospy.spin()
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        setup_walls()
        rate.sleep()


if __name__ == '__main__':
    main()
