#!/usr/bin/env python

import rospy
import math
import numpy as np
import time
from gps_reader.msg import GPS_data
from std_msgs.msg import Int64MultiArray, Float32
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

state_asv = [0.0, 0.0, 0.0] # x, y, theta
state_ref = [2.0, 0.0, 0.0] # x_ref, y_ref, theta_ref
v_asv = [0.0, 0.0, 0.0] # x_ve;, y_vel, ang_course
current = [0.0, 0.0] #current_velocity, current_angle
wayPoints = [] # list of waypoints
target_index = 0 # index of target way point in the wayPoints
h = 0.2
vref = 0.0
ranges = [0]
lidar_inc = 1000
ADCP_mean = [0.0, 0.0, 0.0] # [sum of average, num_samples, mean]


def angleDiff(angle):
    while angle > math.pi:
        angle = angle - 2 * math.pi
    while angle < -math.pi:
        angle = angle + 2 * math.pi
    return angle

def d2t():
    global state_asv, state_ref
    return math.sqrt((state_asv[0] - state_ref[0])**2 +
                    (state_asv[1] - state_ref[1])**2)

def calc_errors():
    global current, state_asv, state_ref, v_asv

    # update the param for live change from the user
    V_REF = 0.5
    VEL_THRESHOLD = rospy.get_param('/v_threshold', 0.2)
    v_ref = 0.5

    '''transform velocities to robots coordinate system'''
    rot = np.array([[np.cos(state_asv[2]), np.sin(state_asv[2])],
        [-np.sin(state_asv[2]), np.cos(state_asv[2])]])
    vel_unrot = np.array([[v_asv[0]],[v_asv[1]]])
    vel_robot = np.matmul(rot, vel_unrot)
    vel_robotX = vel_robot[0]

    lin_v = False

    dist_threshold = rospy.get_param('/dist_threshold', 1.0)

    '''evaluate rotation of robot compared to current'''
    e_heading = angleDiff(current[1] - state_asv[2])
    rospy.logdebug(e_heading)
    ang_dir = state_asv[2]
    # ang_ref = ang_dir

    v = math.sqrt(v_asv[0]**2 + v_asv[1]**2)

    if abs(e_heading) > math.pi/3 and current[0] > 0.1:
        rospy.logdebug('FORSTA')
        # v_ref = vel_robot[0,0] + 0.5
        '''turn robot back towards current if it points to much downward'''
        e_ang = e_heading
        ang_ref = current[1]

    elif d2t() < 0.3:
        e_ang = e_heading
        v_ref = 0.0
        ang_ref = current[1]
    else:
        rospy.logdebug('TREDJE')
        '''desired angle and velocity of robot'''
        des_angle = math.atan2(state_ref[1] - state_asv[1], \
                        state_ref[0] - state_asv[0])
        v = math.sqrt(v_asv[0]**2 + v_asv[1]**2)
        ang_ref = des_angle

        '''desicion on which feedback to use for angle'''
        if v < VEL_THRESHOLD or vel_robot[0,0] < 0.0:
            '''if moving slowly or backwards, use heading as feedback'''
            ang_dir = state_asv[2]
        else:
            '''else use GPS'''
            ang_dir = v_asv[2]

        if d2t() < rospy.get_param('/d2t', 1.5):
            lin_factor = (d2t() - dist_threshold)/rospy.get_param('/d2t', 1.5)
            v_ref = lin_factor * v_ref
            lin_v = True
            print('in lin', lin_factor)
            print('wtf')
            print('')
            print('WTF')


        if abs(angleDiff(des_angle - current[1])) > math.pi/2 \
                            and current[0] > 0.2:
            '''if goal point is downstream go towards it by floating with current'''
            des_angle_downstream = des_angle
            des_angle = angleDiff(2*current[1] - math.pi - des_angle)

            if lin_v:
                des_angle = np.clip(des_angle, current[1] - lin_factor*math.pi/2,
                                        current[1] + lin_factor*math.pi/2)
            ang_ref = des_angle

            v_ref = -abs(0.5 * v_ref * math.cos(current[1] - des_angle_downstream))
            v_ref_alt = -abs(0.5 * v_ref * math.sin(current[1] - des_angle_downstream))
            print('v_ref', v_ref)

            '''transform velocities to error coordinate frame'''

        e_ang = angleDiff(des_angle - ang_dir)

    e_v = v_ref - vel_robot[0,0]
    e_v_alt = v_ref - vel_robot[0,0]

    return e_v, e_ang, e_v_alt, ang_ref, e_heading


def GPS_callb(msg):
    global state_asv, v_asv
    state_asv[0] = msg.x
    state_asv[1] = msg.y
    # velocity
    v_asv[0] = msg.x_vel
    v_asv[1] = msg.y_vel
    v_asv[2] = msg.ang_course

def IMU_callb(msg):
    global state_asv
    state_asv[2] = msg.data

def ADCP_callb2(msg):
    global current, ADCP_mean, state_asv
    data = msg.data

    v_bt = np.array([s16(v) for v in data[7:11]]) # bottom track velocity (v of boat?)
    v_rel_surface = np.array([s16(v) for v in data[11:15]]) # relative surface velocity
    v_surface = v_rel_surface - v_bt
    v_angle = math.atan2(v_surface[1], v_surface[0])

    adcp_offset = rospy.get_param('/ADCP/angleOff', 300.0) / 180.0 * math.pi +math.pi
    v_angle = angleDiff(v_angle+adcp_offset)
    current[0] = math.sqrt(v_surface[0]**2 + v_surface[1]**2) / 1000.0
    current[1] = v_angle

def navGoal_callb(msg):
    ''' reference state update. Specifically for Rviz'''
    global wayPoints, state_ref
    wayPoints = []

    point = msg.pose.position
    state_ref[0] = point.x
    state_ref[1] = point.y
    state_ref[2] = point.z

    wp = GPS_data()
    wp.x = point.x
    wp.y = point.y
    wayPoints.append(point)

def lidar_callb(msg):
    global ranges, lidar_inc
    ranges = np.array(msg.ranges)
    lidar_inc = 2*math.pi/len(ranges)
    trans_broadcast()

def s16(value):
    return -(value & 0x8000) | (value & 0x7fff)


def main():
    rospy.init_node('calc_error')
    rospy.Subscriber('GPS/xy_coord', GPS_data, GPS_callb)
    rospy.Subscriber('heading', Float32, IMU_callb)
    # rospy.Subscriber('ControlCenter/gps_wp', GPS_WayPoints, WP_callb)
    rospy.Subscriber('adcp/data', Int64MultiArray, ADCP_callb2) # needs fixing for real thing
    # rospy.Subscriber('adcp/data/sim', Float32MultiArray, ADCP_callb) # needs fixing for real thing
    rospy.Subscriber('move_base_simple/goal', PoseStamped, navGoal_callb)
    rospy.Subscriber('/os1/scan', LaserScan, lidar_callb)
    start_time = rospy.get_rostime()
    rate = rospy.Rate(1/h)

    errors = np.array([[0],[0], [0], [0], [0], [0]])

    while not rospy.is_shutdown():
        [e_v, e_ang, e_v_alt, ang_ref, e_h] = calc_errors()
        e = np.array([[e_v], [e_ang], [e_v_alt], [ang_ref], [e_h], [(rospy.get_rostime() - start_time).to_sec()]])
        errors = np.append(errors, e, axis=1)
        print('errors', e)
        np.save('/home/filip/Documents/SURF/asv_ws/src/bagfiles/kern/long_run/errors', errors)
        rate.sleep()


if __name__ == '__main__':
    main()
