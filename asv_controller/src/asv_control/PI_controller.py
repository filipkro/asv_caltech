#!/usr/bin/env python

import rospy
import math
import numpy as np
import time

# TODO: Fix controller when reference completed
#       Tune params
#       Add D-part?

#Which global needed?
I_thrust = 0.0
I_rudder = 0.0
h = 0.2

state_asv = [0.0, 0.0, 0.0] # x, y, theta
state_ref = [0.0, 0.0, 0.0] # x_ref, y_ref, theta_ref
v_asv = [0.0, 0.0, 0.0] # x_ve;, y_vel, ang_course
wayPoints = [] # list of waypoints
target_index = 0 # index of target way point in the wayPoints

def update_variable(s_asv, s_ref, v, target_i, wP):
    '''get the update coming from the master '''
    global state_asv, state_ref, v_asv, target_index, wayPoints
    state_asv = s_asv
    state_ref = s_ref
    v_asv = v
    target_index = target_i
    wayPoints = wP

def angleDiff(angle):
    while angle > math.pi:
        angle = angle - 2 * math.pi
    while angle < -math.pi:
        angle = angle + 2 * math.pi
    return angle

def calc_control():
    global state_asv, state_ref, wayPoints, target_index, v_asv
    DIST_THRESHOLD = 1
    '''Fix distance threshold and reference points'''
    rospy.logdebug("Desired wp: " + str(state_ref))
    rospy.logdebug("vel(x,y): " + str(v_asv))

    # maybe don't need this
    # point = wayPoints[target_index]
    # x_ref = point.x
    # y_ref = point.y

    u_thrust = thrust_control(v_asv)
    u_rudder = rudder_control(state_asv, state_ref, v_asv)

    return u_thrust, u_rudder

def thrust_control(v_asv):
    global I_thrust, h
    ''' PI controller. Controls thrust to keep contant velocity.
    Should work in different currents? '''
    ##########################
    ### Control parameters ###
    MAX_THRUST = 1000
    MIN_THRUST = -1000
    '''controller on the form U(s) = K(1 + 1/(Ti*s))*E(s)'''
    K = rospy.get_param('thrust/K', 10.0)
    Ti = rospy.get_param('thrust/Ti', 1.0)
    Tr = Ti/2
    v_ref = rospy.get_param('v_ref', 0.0)
    ##########################

    v = math.sqrt(v_asv[0]**2 + v_asv[1]**2)
    e_v = v_ref - v
    rospy.logdebug("ev: " + str(e_v))
    '''Forward difference discretized PI'''
#    u = K*e_v + K/Ti * I_thrust
    '''saturation'''
#    u_thrust = np.clip(u, MIN_THRUST, MAX_THRUST)
    '''tracking, works as anti-windup. See http://www.control.lth.se/fileadmin/control/Education/EngineeringProgram/FRTN01/2019_lectures/L08_slides6.pdf
    pg. 40-47 for details'''
   # I_thrust = I_thrust + e_v * h + h/Tr*(u_thrust - u)
    u = np.clip(K*(e_v + h/Ti*I_thrust), MIN_THRUST, MAX_THRUST)
    if u >= MIN_THRUST + 50  and u <= MAX_THRUST - 50:
       I_thrust = I_thrust + e_v

    rospy.logdebug("I_thrust: " + str(I_thrust))
    return u

def rudder_control(state_asv, state_ref, v_asv):
    global I_rudder, h
    ##########################
    ### Control parameters ###
    MAX_RUDDER = 1834
    MIN_RUDDER = 1195
    VEL_THRESHOLD = rospy.get_param('/vel_threshold', -1)
    '''controller on the form U(s) = K(1 + 1/(Ti*s))*E(s)'''
    K = rospy.get_param('rudder/K', 1.0)
    Ti = rospy.get_param('rudder/Ti', 1.0)
    Tr = Ti/2
    ##########################
    rospy.logdebug("RUDDER:")
    rospy.logdebug("K: " + str(K))
    rospy.logdebug("Ti: "+ str(Ti))
    

    des_angle = math.atan2(state_ref[1] - state_asv[1], state_ref[0] - state_asv[0])
    v = math.sqrt(v_asv[0]**2 + v_asv[1]**2)
    ''' angle error based on angle course or heading angle?
        With ang_course might be possible with one controller?
        But might be very sensitive, I'll try it! '''
    if v < VEL_THRESHOLD:
        e_ang = angleDiff(des_angle - state_asv[2])
    else:
        e_ang = angleDiff(des_angle - v_asv[2])

    rospy.logdebug("des ang " + str(des_angle))
    rospy.logdebug('ang_course ' + str(v_asv[2]))
    rospy.logdebug("theta " + str(state_asv[2]))
    rospy.logdebug("e ang " + str(e_ang))

#alt 1
    '''Forward difference discretized PI'''
    #u = -(K*e_ang + K/Ti * I_rudder) + 1600
#    u = -(K*e_ang + I_rudder) + 1550
    '''saturation'''
#    u_rudder = np.clip(u, MIN_RUDDER, MAX_RUDDER)
    '''tracking, works as anti-windup. See http://www.control.lth.se/fileadmin/control/Education/EngineeringProgram/FRTN01/2019_lectures/L08_slides6.pdf
    pg. 40-47 for details'''
#    I_rudder = I_rudder +  K*h*(e_ang/Ti + (u_rudder - u)/Tr)
#alt 2
    u = np.clip(-K*(e_ang + h/Ti*I_rudder) + 1600.0, MIN_RUDDER, MAX_RUDDER)
    if u >= MIN_RUDDER + 50  and u <= MAX_RUDDER - 50:
        I_rudder = I_rudder + e_ang

    rospy.logdebug("I_rudder: "+ str(I_rudder))
    return int(u)
    '''set Tr to 1 and move K/Ti to calculation of u??'''
