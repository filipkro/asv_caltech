#!/usr/bin/env python

import rospy
import PI_controller
import transect_controller
from Smart_LIDAR_controller import Smart_LiDAR_Controller
from gps_reader.msg import GPS_data, GPS_WayPoints
from motor_control.msg import MotorCommand
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray # for simulation

state_asv = [0.0, 0.0, 0.0] # x, y, theta
state_ref = [2.0, 0.0, 0.0] # x_ref, y_ref, theta_ref
v_asv = [0.0, 0.0, 0.0] # x_ve;, y_vel, ang_course
current = [0.0, 0.0] #current_velocity, current_angle
wayPoints = [] # list of waypoints
target_index = 0 # index of target way point in the wayPoints
h = 0.2
vref = 0.0
ranges = []
lidar_inc = 0.0

# controllers
smart_controller = Smart_LiDAR_Controller()

########################
## Callback funcitons ##
########################

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
    global wayPoints, state_ref, target_index
    wayPoints = msg.gps_wp
    rospy.loginfo(wayPoints)
    state_ref[0] = wayPoints[0].x
    state_ref[1] = wayPoints[1].y
    target_index = 0

def IMU_callb(msg):
    global state_asv
    state_asv[2] = msg.data

def ADCP_callb(msg): # simulation, not accurate
    global current
    current = msg.data

def ADCP_callb2(msg):
    global current, ADCP_mean
    data = msg.data
    calc_mean = rospy.get_param('ADCP/mean', False)

    v_bt = np.array([s16(v) for v in data[7:11]]) # bottom track velocity (v of boat?)
    v_rel_surface = np.array([s16(v) for v in data[11:15]]) # relative surface velocity
    v_surface = v_rel_surface - v_bt
    v_angle = math.atan2(v_surface[1], v_surface[0])

    adcp_offset = rospy.get_param('/ADCP/angleOff', 135)
    v_angle = angleDiff(v_angle+adcp_offset)

    if calc_mean:
        if rospy.get_param('/ADCP/reset', False):
            ADCP_mean = [0.0, 0.0, 0.0] # [sum of average, num_samples, mean]
            rospy.set_param('/ADCP/reset', False)

        ADCP_mean[0] += v_angle
        ADCP_mean[1] += 1
        ADCP_mean[2] = ADCP_mean[0]/ADCP_mean[1]


def lidar_callb(msg):
    global ranges, lidar_inc
    ranges = np.array(msg.ranges)
    lidar_inc = 2*math.pi/len(ranges)


def updateTarget():
    '''Update target way point based on current position
        return True when there's still point to navigate
        '''
    global target_index
    dist_2_target = math.sqrt( (state_asv[0] - state_ref[0])**2
        + (state_asv[1] - state_ref[1])**2 )

    DIST_THRESHOLD = rospy.get_param('/dist_threshold', 1.0)
    control_mode = rospy.get_param('/nav_mode', 'Waypoint')
    # if we're close to our target then do different things depends on which 
    # controller we're running
    if (dist_2_target <= DIST_THRESHOLD and control_mode != 'Smart'):
        if control_mode == 'Waypoint':
            # for way points, simply iterate through the points and stop
            if (target_index < len(wayPoints)-1):
                target_index += 1
                state_ref[0] = wayPoints[target_index].x
                state_ref[1] = wayPoints[target_index].y
                return True
            else:
                rospy.loginfo('Destination reached')
                return False
        elif control_mode == 'Transect':
            if (len(wayPoints) <= 1):
                rospy.loginfo('Not enough points for transect')
                rospy.set_param('/nav_mode', 'Waypoint')
            elif (target_index == 0):
                # Repeat between the first two points
                target_index = 1
                state_ref[0] = wayPoints[target_index].x
                state_ref[1] = wayPoints[target_index].y
                return True
            else:
                target_index = 0
                state_ref[0] = wayPoints[target_index].x
                state_ref[1] = wayPoints[target_index].y
                return True
            # add something here that'll switch transect back to waypoints once
            # certain time elapsed
        else:
            # don't run if we don't know what controller it is
            return False
    else:
        # smart mode can handle things on its own
        return True


def create_wpList():
    ''' Dummy waypoint generation '''
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

def navGoal_callb(msg):
    ''' reference state update. Specifically for Rviz'''
    global wayPoints
    point = msg.pose.position
    state_ref[0] = point.x
    state_ref[1] = point.y
    state_ref[2] = point.z
    #print("WP added! ", point)
    #print("state ref:", state_ref)
    #print(wayPoints)

def switchControl():
    '''Choose the right controller'''
    global smart_controller, ranges, lidar_inc
    controller_type = rospy.get_param('/nav_mode', 'Waypoint')
    if (controller_type == "Waypoint"):
        return PI_controller.PI_controller()
    elif (controller_type == "Transect"):
        return transect_controller.Transect_controller()
    elif (controller_type == "Smart"):
        smart_controller.update_lidar(ranges, lidar_inc) # smart controlelr needs lidar
        return smart_controller
    else:
        return PI_controller.PI_controller()

def main():
    global samp_time, wayPoints, h, vref, current
    rospy.init_node('master_controller')

    # information update subscriber
    rospy.Subscriber('GPS/xy_coord', GPS_data, GPS_callb)
    rospy.Subscriber('heading', Float32, IMU_callb)
    rospy.Subscriber('ControlCenter/gps_wp', GPS_WayPoints, WP_callb)
    rospy.Subscriber('adcp/data', Float32MultiArray, ADCP_callb) # needs fixing for real thing
    rospy.Subscriber('move_base_simple/goal', PoseStamped, navGoal_callb)
    rospy.Subscriber('/os1/scan', LaserScan, lidar_callb)

    # publish to motor controller
    motor_cmd = MotorCommand()
    ctrl_pub = rospy.Publisher('motor_controller/motor_cmd_reciever', MotorCommand, queue_size=1)

    rate = rospy.Rate(1/h)
    create_wpList()

    while not rospy.is_shutdown():
        # Master control param
            # /run: to run or not to run
            # /control_type: WayPoint, Transect, or something else?
        run = rospy.get_param('/run', False)
        controller = switchControl()
        rospy.logdebug('Target Index '+ str(target_index))
        trgt_updated = updateTarget()

        if run or trgt_updated:
            controller.destinationReached(not trgt_updated)
            controller.update_variable(state_asv, state_ref, v_asv, target_index, wayPoints, current)
            u_thrust, u_rudder = controller.calc_control()
            motor_cmd.port = u_thrust
            motor_cmd.strboard = u_thrust
            motor_cmd.servo = u_rudder
        else:
            controller.destinationReached(not trgt_updated)
            u_thrust, u_rudder = controller.calc_control()
            motor_cmd.port = u_thrust
            motor_cmd.strboard = u_thrust
            motor_cmd.servo = u_rudder

        rospy.logdebug('MotorCmd ' + str(motor_cmd))
        ctrl_pub.publish(motor_cmd)

        rate.sleep()
