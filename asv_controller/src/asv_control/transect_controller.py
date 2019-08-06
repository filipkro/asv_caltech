#!/usr/bin/env python
import rospy
import math
import numpy as np
# transect parameters
transect_p1 = None
trnasect_p2 = None
last_point = None
K_v = 10 # vertical speed gain, displacement to speed
K_latAng = 0.5 # lateral speed to angle
Kp_turn = 200 # turning gain for heading control
v_update_count = 0
ang_update_count = 0
last_u_nom = 0
last_v_des = 0
last_ang_des = 0

state_asv = [0.0, 0.0, 0.0] # x, y, theta
state_ref = [0.0, 0.0, 0.0] # x_ref, y_ref, theta_ref
v_asv = [0.0, 0.0, 0.0] # x_ve;, y_vel, ang_course
wayPoints = [] # list of waypoints
target_index = 0 # index of target way point in the wayPoints

def update_variable(s_asv, s_ref, v, target_i, wP, curr):
    '''get the update coming from the master '''
    global state_asv, state_ref, v_asv, target_index, wayPoints
    state_asv = s_asv
    state_ref = s_ref
    v_asv = v
    target_index = target_i
    wayPoints = wP

def calc_control():
    global v_update_count, ang_update_count
    global last_ang_des, last_u_nom, last_v_des, last_point
    global state_asv, state_ref, wayPoints, v_asv, target_index
    v_x_des = rospy.get_param('/transect/speed_ref', 0.5)
    DIST_THRESHOLD = 1

    if (target_index == 1):
        # simply switch way points between the first two
        last_point = [wayPoints[0].x, wayPoints[0].y, 0.0]
        state_ref = [wayPoints[1].x, wayPoints[1].y, 0.0]
    else:
        last_point = [wayPoints[1].x, wayPoints[1].y, 0.0]
        state_ref = [wayPoints[0].x, wayPoints[0].y, 0.0]

    rospy.logdebug('Last point ' + str(last_point))
    rospy.logdebug('Point now ' + str(state_ref))


    u_rudder = 1600
    v_update_rate = 1
    ang_update_rate = 1
    if v_update_count >= v_update_rate:
        v_vert, u_nom = vertical_speed_control(state_ref)
        last_v_des = v_vert
        last_u_nom = u_nom
        v_update_count = 0
    else:
        v_update_count += 1
        v_vert = last_v_des
        u_nom = last_u_nom

    if ang_update_count >= ang_update_rate:
        ang_des = calc_lateral_ang(state_ref, v_x_des)
        last_ang_des = ang_des
        ang_update_count = 0
    else:
        ang_update_count += 1
        ang_des = last_ang_des

    u_rudder = heading_control(ang_des)
    u_nom = np.clip(u_nom, -1000.0, 1000.0)
    return u_nom, u_rudder

def vertical_speed_control(state_ref):
    '''calculate current velocity toward the waypoint, increase or decrease u_nom'''
    global K_v, last_point, v_asv, state_asv
    K_v = rospy.get_param('/transect/K_v', 200)
    # Calculate drift away from the line
    cur_point = last_point
    next_point = state_ref

    line_angle = math.atan2((state_ref[1] - last_point[1]),(state_ref[0] - last_point[0]))
    position_angle = math.atan2((state_asv[1] - last_point[1]), (state_asv[0] - last_point[0]))

    # calculate position from the line
    pos_ang_from_line = angleDiff(position_angle - line_angle) # position vector from line
    dist = math.sqrt((state_asv[1] - last_point[1])**2 + (state_asv[0] - last_point[0])**2 )
    drift_distance = dist * math.sin(pos_ang_from_line)

    # calculate velocity away from the line
    v_ang_from_line = angleDiff(state_asv[2] - line_angle) # v_vector and line
    drift_v = v_asv[2] * math.sin(v_ang_from_line)

    # heading vector from line
    heading_from_line = angleDiff(state_asv[2] - line_angle)

    # thrust direction
    thrust_dir = heading_from_line/abs(heading_from_line)

    # adding an integral term to remove error (ignore for now)
    # drift_error_integral = self.drift_error_integral + drift_distance * self.dt
    K = rospy.get_param('thrust/K', 10.0)
    v_correct = -drift_distance * K_v #- self.drift_error_integral * self.K_vi
    u_nom = (v_correct + drift_v) * thrust_dir * K

    rospy.logdebug("Drift dist " + str(drift_distance))
    rospy.logdebug("Drift v " + str(drift_v))
    rospy.logdebug("V correct " + str(v_correct))
    rospy.logdebug("u_nom " + str(u_nom))
    rospy.logdebug("thrust_dir " + str(thrust_dir))


    return v_correct, u_nom

def calc_lateral_ang(des_point, v_x_des):
    '''Adjust the ASV angle according to the desired transect (x) speed'''
    global last_point, K_latAng
    K_latAng = rospy.get_param('/transect/K_latAng', 0.5)

    # Calculate drift away from the line
    cur_point = last_point
    next_point = des_point

    line_angle = math.atan2((des_point[1] - last_point[1]),(des_point[0] - last_point[0]))
    v_ang_from_line = angleDiff(v_asv[2] - line_angle)

    v_course = math.sqrt(v_asv[0]**2 + v_asv[1]**2)
    v_x = v_course * math.cos(v_ang_from_line) # this v_x is along the line

    # Calculate heading difference between robot and the line
    heading_from_line = angleDiff(state_asv[2] - line_angle)

    if heading_from_line < 0:
        des_line_heading = angleDiff(-(v_x - v_x_des) * K_latAng + heading_from_line)
    else:
        des_line_heading = angleDiff((v_x - v_x_des) * K_latAng + heading_from_line)


    new_ang = angleDiff(des_line_heading + line_angle)
    rospy.logdebug('V_x ' + str(v_x))
    rospy.logdebug('V_x error' + str(v_x - v_x_des))
    rospy.logdebug('line angle ' + str(line_angle))
    rospy.logdebug('heading ' + str(state_asv[2]))
    rospy.logdebug('heading from lne ' + str(heading_from_line))
    rospy.logdebug('des heading from line ' + str(des_line_heading))
    rospy.logdebug("Ang des " + str(new_ang))
    # print("V_x Difference ", v_x - v_x_des)
    # print("Desired Angle ", new_ang)
    # print("Current Angle ", self.state_est.theta)
    return new_ang

def heading_control(heading_des):
    '''Orient the robot heading by turning the rudder'''
    global Kp_turn
    Kp_turn = rospy.get_param('/transect/Kp_turn', 200)
    u_rudder = -angleDiff(heading_des - state_asv[2]) * Kp_turn
    u_rudder = int(u_rudder + 1600)
    if (u_rudder > 1834):
        u_rudder = 1834
    elif (u_rudder < 1195):
        u_rudder = 1195
    rospy.logdebug("rudder " +  str(u_rudder))
    return u_rudder

def angleDiff(angle):
    while angle > math.pi:
        angle = angle - 2 * math.pi
    while angle < -math.pi:
        angle = angle + 2 * math.pi
    return angle
