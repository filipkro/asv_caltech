#!/usr/bin/env python

import rospy
import PI_controller
import transect_controller
from gps_reader.msg import GPS_data, GPS_WayPoints
from motor_control.msg import MotorCommand
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32
import math
from visualization_msgs.msg import Marker, MarkerArray # for simulation

state_asv = [0.0, 0.0, 0.0] # x, y, theta
state_ref = [0.0, 0.0, 0.0] # x_ref, y_ref, theta_ref
v_asv = [0.0, 0.0, 0.0] # x_ve;, y_vel, ang_course
wayPoints = [] # list of waypoints
target_index = 0 # index of target way point in the wayPoints
h = 0.2

def GPS_callb(msg):
    global state_asv, v_asv
    # position
    state_asv[0] = msg.x
    state_asv[1] = msg.y
    # velocity
    v_asv[0] = msg.x_vel
    v_asv[1] = msg.y_vel
    v_asv[2] = msg.ang_course

    vel = math.sqrt(v_asv[0]**2 + v_asv[1]**2)

def WP_callb(msg):
    global wayPoints, state_ref
    wayPoints = msg.gps_wp
    rospy.loginfo(wayPoints)
    state_ref[0] = wayPoints[0].x
    state_ref[1] = wayPoints[1].y

def IMU_callb(msg):
    global state_asv
    state_asv[2] = msg.data

def updateTarget():
    '''Update target way point base on current position
        return True when there's still point to navigate
        '''
    global target_index
    dist_2_target = math.sqrt( (state_asv[0] - state_ref[0])**2 
        + (state_asv[1] - state_ref[1])**2 )

    DIST_THRESHOLD = rospy.get_param('/dist_threshold', 1.0)
    if (dist_2_target <= DIST_THRESHOLD):
        if rospy.get_param('/nav_mode') == 'Waypoint':
            # for way points, simply iterate through the points and stop
            if (target_index < len(wayPoints)-1):
                target_index += 1
                state_ref[0] = wayPoints[target_index].x
                state_ref[1] = wayPoints[target_index].y
                return True
            else:
                rospy.loginfo('Destination reached')
                return False
        elif rospy.get_param('/nav_mode') == 'Transect':
            # for transect, just repeat. Only take first two points
            if (target_index == 0):
                target_index = 1
                state_ref[0] = wayPoints[target_index].x
                state_ref[1] = wayPoints[target_index].y
                return True
            else:
                target_index = 0
                state_ref[0] = wayPoints[target_index].x
                state_ref[1] = wayPoints[target_index].y
                return True
        else:
            # don't run if we don't know what controller it is
            return False

    else:
        return True
    

def create_wpList():
    global wayPoints
    pub = rospy.Publisher('wpArray', MarkerArray, queue_size=1)
    markerArray = MarkerArray()

    point = GPS_data()
    list = []
    point.x = 1.0
    point.y = 0.0
    wayPoints.append(point)

    point = GPS_data()
    point.x = 5.0
    point.y = 2.0
    wayPoints.append(point)

    point = GPS_data()
    point.x = 70.0
    point.y = 60.0
    wayPoints.append(point)

    point = GPS_data()
    point.x = 50.0
    point.y = 50.0
    wayPoints.append(point)

    point = GPS_data()
    point.x = 30.0
    point.y = 20.0
    wayPoints.append(point)

    point = GPS_data()
    point.x = 0.0
    point.y = 0.0
    wayPoints.append(point)

def switchControl():
    controller_type = rospy.get_param('/nav_mode', 'Waypoint') 
    if (controller_type == "Waypoint"):
        return PI_controller
    elif (controller_type == "Transect"):
        return transect_controller
    else:
        return PI_controller

def main():
    global samp_time, wayPoints, h
    rospy.init_node('master_controller')
    
    # information update subscriber
    rospy.Subscriber('GPS/xy_coord', GPS_data, GPS_callb)
    rospy.Subscriber('heading', Float32, IMU_callb)
    rospy.Subscriber('ControlCenter/gps_wp', GPS_WayPoints, WP_callb)
    
    # publish to motor controller
    motor_cmd = MotorCommand()
    ctrl_pub = rospy.Publisher('motor_controller/motor_cmd_reciever', MotorCommand, queue_size=1)

    rate = rospy.Rate(1/h)
    create_wpList()
    # print(wayPoints)

    while not rospy.is_shutdown():
        # Master control param 
            # /run: to run or not to run
            # /control_type: WayPoint, Transect, or something else?
        run = rospy.get_param('/run', False)
        controller = switchControl()
        rospy.logdebug('Target Index '+ str(target_index))
        if run or updateTarget():
            controller.update_variable(state_asv, state_ref, v_asv, target_index, wayPoints)
            u_thrust, u_rudder = controller.calc_control()
            motor_cmd.port = u_thrust
            motor_cmd.strboard = u_thrust
            motor_cmd.servo = u_rudder
        else:
            motor_cmd.port = 0.0
            motor_cmd.strboard = 0.0
            motor_cmd.servo = 1600

        # print(motor_cmd)
        ctrl_pub.publish(motor_cmd)

        rate.sleep()

