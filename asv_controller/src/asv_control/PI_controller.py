#!/usr/bin/env python

import rospy
import math
import numpy as np
import time

# TODO: Fix controller when reference completed
#       Tune params
#       Add D-part?


'''Changes needed in master:
    - ADCP callback which converts to angle and velocity (not needed at this point though)
        passed to this as current in update_variable
    -
'''



#Which global needed?
I_thrust = 0.0
I_rudder = 0.0
h = 0.2

state_asv = [0.0, 0.0, 0.0] # x, y, theta
state_ref = [0.0, 0.0, 0.0] # x_ref, y_ref, theta_ref
current = [0.0, 0.0] #current_angle, current_velocity
v_asv = [0.0, 0.0, 0.0] # x_vel;, y_vel, ang_course
wayPoints = [] # list of waypoints
target_index = 0 # index of target way point in the wayPoints
destReached = True # destination reached? i.e. true if no more points in list


def update_variable(s_asv, s_ref, v, target_i, wP, curr):
    '''get the update coming from the master '''
    global state_asv, state_ref, v_asv, target_index, wayPoints, current
    state_asv = s_asv
    state_ref = s_ref
    v_asv = v
    target_index = target_i
    wayPoints = wP
    current = curr

def destinationReached(destBool):
    global destReached
    destReached = destBool

def angleDiff(angle):
    while angle > math.pi:
        angle = angle - 2 * math.pi
    while angle < -math.pi:
        angle = angle + 2 * math.pi
    return angle

def calc_control():
    global state_asv, state_ref, wayPoints, target_index, v_asv, current, destReached
    rospy.logdebug("Desired wp: " + str(state_ref))
    rospy.logdebug("vel(x,y): " + str(v_asv))

    v_ref = rospy.get_param('/v_ref', 0.0)
    VEL_THRESHOLD = rospy.get_param('/v_threshold', 0.2)


    '''transform velocities to robots coordinate system'''
    rot = np.array([[np.cos(state_asv[2]), np.sin(state_asv[2])], \
        [-np.sin(state_asv[2]), np.cos(state_asv[2])]])
    vel_unrot = np.array([[v_asv[0]],[v_asv[1]]])
    vel_robot = np.matmul(rot, vel_unrot)

    '''evaluate rotation of robot compared to current'''
    e_heading = angleDiff(current[0] + math.pi - state_asv[2])


    if abs(e_heading) > math.pi/3 and current[1] > 0.1:
        print('FORSTA')
        print(e_heading)
        '''turn robot back towards current if it points to much downward'''
        e_ang = e_heading
    elif destReached:
        print('ANDRA')
        '''if completed turn boat towards current, keep velocity at 0.0
            reason not to have this if statement first is thrust is needed to rotate boat'''
        e_ang = e_heading
        v_ref = 0.0
    else:
        print('TREDJE')
        '''desired angle and velocity of robot'''
        des_angle = math.atan2(state_ref[1] - state_asv[1], state_ref[0] - state_asv[0])
        v = math.sqrt(v_asv[0]**2 + v_asv[1]**2)

        '''desicion on which feedback to use for angle'''
        if v < VEL_THRESHOLD or vel_robot[0,0] < 0.0:
            '''if moving slowly or backwards, use heading as feedback'''
            ang_dir = state_asv[2]
        else:
            '''else use GPS'''
            ang_dir = v_asv[2]

        rospy.logdebug("des ang " + str(des_angle))

        if abs(angleDiff(des_angle - current[0])) < math.pi/2 and current[1] > 0.1:
            '''if goal point is downstream go towards it by floating with current'''
            e_ang = angleDiff(math.pi - des_angle + ang_dir)
            v_ref = -v_ref/2
        else:
            e_ang = angleDiff(des_angle - ang_dir)

        if abs(angleDiff(current[0] - state_asv[2])) < 0.05 and abs(current[1] - v_ref) < 0.1:
            v_ref += 0.5

    e_v = v_ref - vel_robot[0,0]
    u_rudder = rudder_control(e_ang)
    u_thrust = thrust_control(e_v)
    # maybe don't need this
    # point = wayPoints[target_index]
    # x_ref = point.x
    # y_ref = point.y



    return u_thrust, u_rudder

def thrust_control(e_v):
    global I_thrust, h
    ''' PI controller. Controls thrust to keep contant velocity.
    Should work in different currents? '''
    ##########################
    ### Control parameters ###
    MAX_THRUST = 1000
    MIN_THRUST = -1000
    '''controller on the form U(s) = K(1 + 1/(Ti*s))*E(s)'''
    K = rospy.get_param('thrust/K', 500.0)
    Ti = rospy.get_param('thrust/Ti', 1.0)

    rospy.logdebug("THRUST:")
    rospy.logdebug("K: " + str(K))
    rospy.logdebug("Ti: " + str(Ti))
    ##########################

    u = np.clip(K*(e_v + h/Ti*I_thrust), MIN_THRUST, MAX_THRUST)
    if u >= MIN_THRUST + 50  and u <= MAX_THRUST - 50:
        I_thrust = I_thrust + e_v

    rospy.logdebug("I_thrust: " + str(I_thrust))

    return u

def rudder_control(e_ang):
    global I_rudder, h
    ##########################
    ### Control parameters ###
    MAX_RUDDER = 1834
    MIN_RUDDER = 1195

    '''controller on the form U(s) = K(1 + 1/(Ti*s))*E(s)'''
    K = rospy.get_param('rudder/K', 2000.0)
    Ti = rospy.get_param('rudder/Ti', 10.0)
    ##########################
    rospy.logdebug("RUDDER:")
    rospy.logdebug("K: " + str(K))
    rospy.logdebug("Ti: " + str(Ti))


    rospy.logdebug('ang_course ' + str(v_asv[2]))
    rospy.logdebug("theta " + str(state_asv[2]))
    rospy.logdebug("e ang " + str(e_ang))

    u = np.clip(-K*(e_ang + h/Ti*I_rudder) + 1600.0, MIN_RUDDER, MAX_RUDDER)
    if u >= MIN_RUDDER + 50  and u <= MAX_RUDDER - 50:
        I_rudder = I_rudder + e_ang

    rospy.logdebug("I_rudder: "+ str(I_rudder))
    return int(u)
    '''set Tr to 1 and move K/Ti to calculation of u??'''
