#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from gps_reader.msg import GPS_data
from motor_control.msg import MotorCommand
from geometry_msgs.msg import PointStamped
import math
import numpy as np


# Very similar to PI_controller.py, but angle error generated based on
# direction of boat instead of direction of travel

#Which global needed?
x = 0.0
y = 0.0
theta = 0.0
x_vel = 0.0
y_vel = 0.0
ang_vel = 0.0
dest_points = []
x_ref = 0.0
y_ref = 0.0
I_thrust = 0.0
I_rudder = 0.0
h = 0.2

def GPS_callb(msg):
    global x, y, x_vel, y_vel, ang_course, ang_vel
    x = msg.x
    y = msg.y
    x_vel = msg.x_vel
    y_vel = msg.y_vel
    ang_course = msg.ang_course
    ang_vel = msg.ang_vel

def IMU_callb(msg):
    global theta
    theta = msg.data[8]

def point_callb(msg):
    global dest_points
    point = (msg.point.x, msg.point.y)
    dest_points.append(point)

def angleDiff(angle):
    while angle > math.pi:
        angle = angle - 2 * math.pi
    while angle < -math.pi:
        angle = angle + 2 * math.pi

    return angle


def calc_control():
    global x_ref, y_ref, dest_points
    DIST_THRESHOLD = 0.5
    '''Fix distance threshold and reference points'''
    dist = math.sqrt((x_ref - x)**2 + (y_ref - y)**2)
    if dist <= DIST_THRESHOLD:
        if len(dest_points) == 0:
            ''' Set rudder angle to point boat upstream '''
            ''' Set thrust to keep constant velocity '''
            u_thrust = 0
            u_rudder = 1515
            ''' As of now sets thrust to 0 and rudder straight '''
            return u_thrust, u_rudder
        else:
            x_ref, y_ref = dest_points.pop(0)

    u_thrust = thrust_control(v_ref)
    u_rudder = rudder_control(x_ref, y_ref)

    return u_thrust, u_rudder


def thrust_control(v_ref):
    global x_vel, y_vel, I_thrust, h
    ''' PI controller. Controls thrust to keep contant velocity.
    Should work in different currents? '''
    ##########################
    ### Control parameters ###
    MAX_THRUST = 1000
    MIN_THRUST = 0
    '''controller on the form U(s) = K(1 + 1/(Ti*s))*E(s)'''
    K = 1.0
    Ti = 1.0
    Tr = Ti/2
    ##########################

    v = math.sqrt(x_vel**2 + y_vel**2)
    e_v = v_ref - v
    '''Forward difference discretized PI'''
    u = K*e_v + I_thrust
    '''saturation'''
    u_thrust = np.clip(u, MIN_THRUST, MAX_THRUST)
    '''tracking, works as anti-windup. See http://www.control.lth.se/fileadmin/control/Education/EngineeringProgram/FRTN01/2019_lectures/L08_slides6.pdf
    pg. 40-47 for details'''
    I_thrust = I_thrust + K/Ti * e_v * h + h/Tr*(u_thrust - u)

    return u_thrust


def rudder_control(x_ref, y_ref):
    global x, y, theta, I_rudder, h
    ##########################
    ### Control parameters ###
    MAX_RUDDER = 1834
    MIN_RUDDER = 1195
    '''controller on the form U(s) = K(1 + 1/(Ti*s))*E(s)'''
    K = 1.0
    Ti = 1.0
    Tr = Ti/2
    ##########################

    des_angle = math.atan2(y_ref - y, x_ref - x)
    ''' angle error based on angle course or heading angle?
        With ang_course might be possible with one controller?
        But might be very sensitive, I'll try it! '''
    e_ang = angleDiff(des_angle - theta)
    # alt: e_ang = angleDiff(des_angle - ang)
    '''Forward difference discretized PI'''
    u = K*e_ang + I_rudder + 1600
    '''saturation'''
    u_rudder = np.clip(u, MIN_RUDDER, MAX_RUDDER)
    '''tracking, works as anti-windup. See http://www.control.lth.se/fileadmin/control/Education/EngineeringProgram/FRTN01/2019_lectures/L08_slides6.pdf
    pg. 40-47 for details'''
    I_rudder = I_rudder + K/Ti * e_ang * h + h/Tr*(u_rudder - u)

    return int(u_rudder)


def main():
    global samp_time
    rospy.init_node('main_controller')
    rospy.Subscriber('GPS_coord', GPS_data, GPS_callb)
    rospy.Subscriber('imu', Float32MultiArray, IMU_callb)
    rospy.Subscriber('nav_point', PointStamped, point_callb)
    ''' Subscribing to destination points to add to destination vector.
        What topic? What message?'''
    ctrl_pub = rospy.Publisher('motor_controller/motor_cmd_reciever', MotorCommand, queue_size=1)
    motor_cmd = MotorCommand()
    rate = rospy.Rate(1/h)


    while not rospy.is_shutdown():
        u_thrust, u_rudder = calc_control()
        motor_cmd.port = u_thrust
        motor_cmd.strboard = u_thrust
        motor_cmd.servo = u_rudder
        ctrl_pub.publish(motor_cmd)

        rate.sleep()
