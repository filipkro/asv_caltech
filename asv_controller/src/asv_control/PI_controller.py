#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from gps_reader.msg import GPS_data, GPS_WayPoints
from motor_control.msg import MotorCommand
from geometry_msgs.msg import PointStamped
import math
import numpy as np

# TODO: Fix reference vector/subscriber (!!!)
#       Fix controller when reference completed
#       Tune params
#       Add D-part?

#Which global needed?
x = 0.0
y = 0.0
ang = 0.0
x_vel = 0.0
y_vel = 0.0
ang_course = 0.0
ang_vel = 0.0
wayPoints = []
x_ref = 5.0
y_ref = 4.0
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
    global ang
    ang = msg.data[8]

def WP_callb(msg):
    global wayPoints
    wayPoints = msg
    rospy.loginfo(wayPoints)

def angleDiff(angle):
    while angle > math.pi:
        angle = angle - 2 * math.pi
    while angle < -math.pi:
        angle = angle + 2 * math.pi

    return angle


def calc_control():
    global x_ref, y_ref, wayPoints
    DIST_THRESHOLD = 0.5
    '''Fix distance threshold and reference points'''
    dist = math.sqrt((x_ref - x)**2 + (y_ref - y)**2)
    if dist <= DIST_THRESHOLD:
        print("hej")
        if len(wayPoints) == 0:
            ''' Set rudder angle to point boat upstream '''
            ''' Set thrust to keep constant velocity '''
            u_thrust = 0
            u_rudder = 1515
            ''' As of now sets thrust to 0 and rudder straight '''
            return u_thrust, u_rudder
        else:
             point = wayPoints.pop(0)
             x_ref = point.x
             y_ref = point.y

    u_thrust = thrust_control()
    u_rudder = rudder_control(x_ref, y_ref)

    return u_thrust, u_rudder


def thrust_control():
    global x_vel, y_vel, I_thrust, h
    ''' PI controller. Controls thrust to keep contant velocity.
    Should work in different currents? '''
    ##########################
    ### Control parameters ###
    MAX_THRUST = 1000
    MIN_THRUST = 0
    '''controller on the form U(s) = K(1 + 1/(Ti*s))*E(s)'''
    K = rospy.get_param('thrust/K', 1.0)
    Ti = rospy.get_param('thrust/Ti', 1.0)
    Tr = Ti/2
    ##########################

    print("THRUST:")
    print("K: ", K)
    print("Ti: ", Ti)

    v_ref = rospy.get_param('v_ref', 0.0)

    v = math.sqrt(x_vel**2 + y_vel**2)
    e_v = v_ref - v
    '''Forward difference discretized PI'''
    u = K*e_v + I_thrust
    '''saturation'''
    u_thrust = np.clip(u, MIN_THRUST, MAX_THRUST)
    '''tracking, works as anti-windup. See http://www.control.lth.se/fileadmin/control/Education/EngineeringProgram/FRTN01/2019_lectures/L08_slides6.pdf
    pg. 40-47 for details'''
    I_thrust = I_thrust + K/Ti * e_v * h + h/Tr*(u_thrust - u)
    print("I_thrust: ", I_thrust)
    return u_thrust


def rudder_control(x_ref, y_ref):
    global x, y, ang_course, I_rudder, h
    ##########################
    ### Control parameters ###
    MAX_RUDDER = 1834
    MIN_RUDDER = 1195
    '''controller on the form U(s) = K(1 + 1/(Ti*s))*E(s)'''
    K = rospy.get_param('rudder/K', 1.0)
    Ti = rospy.get_param('rudder/Ti', 1.0)
    Tr = Ti/2
    ##########################
    print("RUDDER:")
    print("K: ", K)
    print("Ti: ", Ti)

    des_angle = math.atan2(y_ref - y, x_ref - x)
    ''' angle error based on angle course or heading angle?
        With ang_course might be possible with one controller?
        But might be very sensitive, I'll try it! '''
    e_ang = angleDiff(des_angle - ang_course)
    # alt: e_ang = angleDiff(des_angle - ang)
    '''Forward difference discretized PI'''
    u = K*e_ang + I_rudder + 1600
    '''saturation'''
    u_rudder = np.clip(u, MIN_RUDDER, MAX_RUDDER)
    '''tracking, works as anti-windup. See http://www.control.lth.se/fileadmin/control/Education/EngineeringProgram/FRTN01/2019_lectures/L08_slides6.pdf
    pg. 40-47 for details'''
    I_rudder = I_rudder + K/Ti * e_ang * h + h/Tr*(u_rudder - u)
    print("I_rudder: ", I_rudder)
    return int(u_rudder)


def main():
    global samp_time
    rospy.init_node('main_controller')
    rospy.Subscriber('GPS_coord', GPS_data, GPS_callb)
    rospy.Subscriber('imu', Float32MultiArray, IMU_callb)
    rospy.Subscriber('ControlCenter/gps_wp', GPS_WayPoints, WP_callb)
    ''' Subscribing to destination points to add to destination vector.
        What topic? What message?'''
    ctrl_pub = rospy.Publisher('motor_controller/motor_cmd_reciever', MotorCommand, queue_size=1)
    motor_cmd = MotorCommand()
    rate = rospy.Rate(1/h)


    while not rospy.is_shutdown():
        run = rospy.get_param('/run', False)
        print(run)
        if run:
            u_thrust, u_rudder = calc_control()
            motor_cmd.port = u_thrust
            motor_cmd.strboard = u_thrust
            motor_cmd.servo = u_rudder
        else:
            motor_cmd.port = 0.0
            motor_cmd.strboard = 0.0
            motor_cmd.servo = 1600

        ctrl_pub.publish(motor_cmd)

        rate.sleep()
