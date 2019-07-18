#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from gps_reader.msg import GPS_data, GPS_WayPoints
from motor_control.msg import MotorCommand
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32
import math
import numpy as np
import time

#for sim
from visualization_msgs.msg import Marker, MarkerArray

# TODO: Fix controller when reference completed
#       Tune params
#       Add D-part?

#Which global needed?
x = 0.0
y = 0.0
theta = 0.0
x_vel = 0.0
y_vel = 0.0
ang_course = 0.0
wayPoints = []
x_ref = 0.0
y_ref = 0.0
I_thrust = 0.0
I_rudder = 0.0
h = 0.2
x_refPrev = 0.0
y_refPrev = 0.0


# transect parameters
transect_p1 = None
trnasect_p2 = None
last_point = None
K_v = 10 # vertical speed gain
K_latAng = 10
Kp_Ang = 10
v_update_count = 0
ang_update_count = 0
last_u_nom = 0
last_v_des = 0
last_ang_des = 0

def GPS_callb(msg):
    global x, y, x_vel, y_vel, ang_course
    x = msg.x
    y = msg.y
    x_vel = msg.x_vel
    y_vel = msg.y_vel
    vel = math.sqrt(x_vel**2 + y_vel**2)
    ang_course = msg.ang_course
    #print(x, y, msg.ang_course, vel)

def IMU_callb(msg):
    global theta
    theta = msg.data
    #print('IMU CALLBACK')
   # print("degree ", msg.data/math.pi * 180)
   # print("radian ", msg.data)
    #print('theta ', theta)

def WP_callb(msg):
    global wayPoints, x_ref, y_ref
    wayPoints = msg.gps_wp
    rospy.loginfo(wayPoints)
    x_ref = wayPoints[0].x
    y_ref = wayPoints[0].y
    # print("callback")
    # print("x_ref: ", x_ref)
    # print("y_ref: ", y_ref)
    # print(wayPoints.gps_wp)
    nav_mode = rospy.get_param('\nav_mode')
        
    if (nav_mode == 'Waypoint'):
        pass
    else:
        transect_p1 = [x_ref, y_ref]
        transect_p2 = [wayPoints[1].x, wayPoints[1].y]
        last_point = transect_p2

def angleDiff(angle):
    while angle > math.pi:
        angle = angle - 2 * math.pi
    while angle < -math.pi:
        angle = angle + 2 * math.pi

    return angle

def transect_control(v_x_des):
    global x_ref, y_ref, wayPoints, x_vel, y_vel, v_update_count, last_v_des, ang_update_count
    global last_ang_des, last_u_nom

    DIST_THRESHOLD = 1
    dist = math.sqrt((x_ref - x)**2 + (y_ref - y)**2)

    if (dist <= DIST_THRESHOLD):
        # simple switching way points between two values
        x_temp = x_ref
        y_temp = y_ref
        x_ref = last_point[0]
        y_ref = last_point[1]
        last_point[0] = x_ref
        last_point[0] = y_ref
    else:
        u_rudder = 1515
        v_update_rate = 5
        ang_update_rate = 5
        if v_update_count >= v_update_rate:
            v_vert, u_nom = vertical_speed_control(des_point)
            last_v_des = v_vert
            last_u_nom = u_nom
            v_update_count = 0
        else:
            v_update_count += 1
            v_vert = last_v_des
            u_nom = last_u_nom

        if ang_update_count >= ang_update_rate:
            ang_des = calc_lateral_ang(des_point, v_x_des)
            last_ang_des = ang_des
            ang_update_count = 0
        else:
            ang_update_count += 1
            ang_des = last_ang_des

        u_rudder = heading_control(ang_des)

    return u_nom, u_rudder

def vertical_speed_control(des_point):
    '''calculate current velocity toward the waypoint, increase or decrease u_nom'''
    global K_v, last_point, ang_course, theta
    # Calculate drift away from the line
    cur_point = last_point
    next_point = [x_ref, y_ref]

    line_angle = math.atan2((y_ref - last_point[1]),(x_ref - last_point[0]))
    position_angle = math.atan2((y - last_point[1]), (x - last_point[0]))

    # calculate position from the line
    pos_ang_from_line = angleDiff(position_angle - line_angle) # position vector from line
    dist = math.sqrt((y - last_point[1])**2 + (x - last_point[0])**2 )
    drift_distance = dist * math.sin(pos_ang_from_line)

    # calculate velocity away from the line
    v_ang_from_line = angleDiff(ang_course - line_angle) # v_vector and line
    drift_v = ang_course * math.sin(v_ang_from_line)

    # heading vector from line
    heading_from_lline = angleDiff(theta - line_angle)

    # thrust direction
    thrust_dir = heading_from_line/abs(heading_from_line)

    # adding an integral term to remove error (ignore for now)
    # drift_error_integral = self.drift_error_integral + drift_distance * self.dt
    K = rospy.get_param('thrust/K', 10.0)

    v_correct = -drift_distance * K_v #- self.drift_error_integral * self.K_vi
    u_nom = (v_correct + drift_v) * thrust_dir * K

    return v_correct, u_nom

def calc_lateral_ang(des_point, v_x_des):
    '''Adjust the ASV angle according to the desired transect (x) speed'''
    global last_point, ang_course, theta, K_latAng

    # Calculate drift away from the line
    cur_point = last_point
    next_point = des_point

    line_angle = math.atan2((des_point[1] - last_point[1]),(des_point[0] - last_point[0]))
    v_ang_from_line = angleDiff(ang_course - line_angle)

    v_course = math.sqrt(x_vel**2 + y_vel**2)
    v_x = v_course * math.cos(v_ang_from_line) # this v_x is along the line

    # Calculate heading difference between robot and the line
    heading_from_line = angleDiff(theta - line_angle)

    if heading_from_line < 0:
        des_line_heading = angleDiff(-(v_x - v_x_des) * K_latAng + heading_from_line)
    else:
        des_line_heading = angleDiff((v_x - v_x_des) * K_latAng + heading_from_line)


    new_ang = angleDiff(des_line_heading + line_angle)
    # print("V_x Difference ", v_x - v_x_des)
    # print("Desired Angle ", new_ang)
    # print("Current Angle ", self.state_est.theta)
    return new_ang

def heading_control(self, heading_des):
    '''Orient the robot heading by turning the rudder'''
    global theta, Kp_Ang
    u_rudder = -angleDiff(heading_des - theta) * Kp_Ang
    u_rudder = int(u_rudder + 1515)
    if (u_rudder > 1834):
        u_rudder = 1834
    elif (u_rudder < 1195):
        u_rudder = 1195

    return u_rudder

def calc_control():
    global x_ref, y_ref, wayPoints, x_vel, y_vel, x, y, x_refPrev, y_refPrev
    DIST_THRESHOLD = 1
    '''Fix distance threshold and reference points'''
    dist = math.sqrt((x_ref - x)**2 + (y_ref - y)**2)
    print('Distance to point', dist)
    print("Desired wp: ", x_ref, y_ref)
    print("vel(x,y): ", x_vel, y_vel)

#######################
    if y_ref - y_refPrev != 0:
        k = (x_refPrev - x_ref)/(y_ref - y_refPrev)
        m = y_ref - k * x_ref
    else:
        k = 1
        m = 100
#######################

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
             x_refPrev = x_ref
             y_refPrev = y_ref
             x_ref = point.x
             y_ref = point.y

    v = math.sqrt(x_vel**2 + y_vel**2)
    u_thrust = thrust_control(v)
   # u_thrust = 0
    u_rudder = rudder_control(x_ref, y_ref, v)

    return u_thrust, u_rudder


def thrust_control(v):
    global x_vel, y_vel, I_thrust, h
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
    ##########################

    print("THRUST:")
    print("K: ", K)
    print("Ti: ", Ti)

    v_ref = rospy.get_param('v_ref', 0.0)


    e_v = v_ref - v
    print("ev: ", e_v)
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

    print("I_thrust: ", I_thrust)
    return u


def rudder_control(x_ref, y_ref, v):
    global x, y, ang_course, theta, I_rudder, h
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
    print("RUDDER:")
    print("K: ", K)
    print("Ti: ", Ti)

    des_angle = math.atan2(y_ref - y, x_ref - x)
    ''' angle error based on angle course or heading angle?
        With ang_course might be possible with one controller?
        But might be very sensitive, I'll try it! '''
    if v < VEL_THRESHOLD:
        e_ang = angleDiff(des_angle - theta)
    else:
        e_ang = angleDiff(des_angle - ang_course)

    print("des ang", des_angle)
    print('ang_course', ang_course)
    print("theta", theta)
    print("e ang", e_ang)

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

    print("I_rudder: ", I_rudder)
    return int(u)
    '''set Tr to 1 and move K/Ti to calculation of u??'''

def create_wpList():
    global wayPoints
    pub = rospy.Publisher('wpArray', MarkerArray, queue_size=1)
    markerArray = MarkerArray()

    point = GPS_data()
    list = []
    point.x = 2.0
    point.y = 0.0
    wayPoints.append(point)

    point = GPS_data()
    point.x = 5.0
    point.y = 2.0
    wayPoints.append(point)

    point = GPS_data()
    point.x = 7.0
    point.y = 6.0
    wayPoints.append(point)

    point = GPS_data()
    point.x = 5.0
    point.y = 5.0
    wayPoints.append(point)

    point = GPS_data()
    point.x = 3.0
    point.y = 2.0
    wayPoints.append(point)

    point = GPS_data()
    point.x = 0.0
    point.y = 0.0
    wayPoints.append(point)

def main():
    global samp_time, wayPoints
    rospy.init_node('main_controller')
    rospy.Subscriber('GPS/xy_coord', GPS_data, GPS_callb)
    rospy.Subscriber('heading', Float32, IMU_callb)
    rospy.Subscriber('ControlCenter/gps_wp', GPS_WayPoints, WP_callb)
    ''' Subscribing to destination points to add to destination vector.
        What topic? What message?'''
    ctrl_pub = rospy.Publisher('motor_controller/motor_cmd_reciever', MotorCommand, queue_size=1)
    motor_cmd = MotorCommand()
    rate = rospy.Rate(1/h)

   # create_wpList()

    print(wayPoints)

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

        print(motor_cmd)
        ctrl_pub.publish(motor_cmd)

        rate.sleep()
