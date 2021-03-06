#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from gps_reader.msg import GPS_data
from motor_control.msg import MotorCommand
from geometry_msgs.msg import PointStamped
import math

#Which global needed?
x = 0.0
y = 0.0
ang = 0.0
x_vel = 0.0
y_vel = 0.0
ang_course = 0.0
ang_vel = 0.0
dest_points = []
x_ref = 0.0
y_ref = 0.0
I_thrust = 0.0
I_rudder = 0.0
samp_time = 0.2

def GPS_callb(msg):
    global x, y, x_vel, y_vel, ang_course, ang_vel
    x = msg.x
    y = msg.y
    x_vel = msg.x_vel
    y_vel = msg.y_vel
    ang_course = msg.ang_course
    ang_vel = msg.ang_vel

def IMU_callb(msg):
    global ang
    ang = msg.data[8]

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
            return u_thrust, u_rudder
        else:
            x_ref, y_ref = dest_points.pop(0)

    u_thrust = thrust_control(v_ref)
    u_rudder = rudder_control(x_ref, y_ref)

    return u_thrust, u_rudder


def thrust_control(v_ref):
    global x_vel, y_vel, I_thrust, samp_time
    ''' PI controller. Controls thrust to keep contant velocity.
    Should work in different currents? '''
    ##########################
    ### Control parameters ###
    MAX_THRUST = 1000
    MIN_THRUST = 0
    Kp_thrust = 1.0
    Ki_thrust = 1.0
    ##########################

    v = math.sqrt(x_vel**2 + y_vel**2)
    e_v = v_ref - v
    I_thrust = I_thrust + Ki_thrust * e_v * samp_time
    '''anti-windup'''
    if I_thrust > MAX_THRUST:
        I_thrust = MAX_THRUST
    elif I_thrust < MIN_THRUST:
        I_thrust = MIN_THRUST

    u_thrust = Kp_thrust*e_v + I_thrust
    '''saturation'''
    if u_thrust > MAX_THRUST:
        u_thrust = MAX_THRUST
    elif u_thrust < MIN_THRUST:
        u_thrust = MIN_THRUST

    return u_thrust


def rudder_control(x_ref, y_ref):
    global x, y, ang_course, I_rudder, samp_time
    ##########################
    ### Control parameters ###
    MAX_RUDDER = 1834
    MIN_RUDDER = 1195
    K_rudder = 1.0
    Ki_rudder = 1.0
    ##########################

    des_angle = math.atan2(y_ref - y, x_ref - x)
    ''' angle error based on angle course or heading angle?
        With ang_course might be possible with one controller?
        But might be very sensitive, I'll try it! '''
    e_ang = angleDiff(des_angle - ang_course)
    # alt: e_ang = angleDiff(des_angle - ang)
    I_rudder = I_rudder + Ki_rudder * e_ang * samp_time

    '''anti-windup'''
    if I_rudder > MAX_RUDDER:
        I_rudder = MAX_RUDDER
    elif I_rudder < MIN_RUDDER:
        I_rudder = MIN_RUDDER

    '''P controller for rudder angle with offset and saturation for servo'''
    u_rudder = int(K_rudder * e_ang + 1600)
    if (u_rudder > MAX_RUDDER):
        u_rudder = MAX_RUDDER
    elif (u_rudder < MIN_RUDDER):
        u_rudder = MIN_RUDDER

    return u_rudder


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
    rate = rospy.Rate(1/samp_time)


    while not rospy.is_shutdown():
        u_thrust, u_rudder = calc_control()
        motor_cmd.port = u_thrust
        motor_cmd.strboard = u_thrust
        motor_cmd.servo = u_rudder
        ctrl_pub.publish(motor_cmd)

        rate.sleep()
