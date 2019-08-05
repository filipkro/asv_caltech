#!/usr/bin/env python

import rospy
import PI_controller
import transect_controller
from gps_reader.msg import GPS_data, GPS_WayPoints
from motor_control.msg import MotorCommand
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Float32, Float32MultiArray, String
import math
from visualization_msgs.msg import Marker, MarkerArray # for simulation

state_asv = [0.0, 0.0, 0.0] # x, y, theta
state_ref = [0.0, 0.0, 0.0] # x_ref, y_ref, theta_ref
v_asv = [0.0, 0.0, 0.0] # x_ve;, y_vel, ang_course
current = [0.0, 0.0] #current_velocity, current_angle
wayPoints = [] # list of waypoints
target_index = 0 # index of target way point in the wayPoints
h = 0.2
ADCP_mean = [0.0, 0.0, 0.0] # for means of ADCP angle, [sum, cnt, mean]
ranges = []

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


'''for mean calculations (and the rest??)
    make sure current angle is global
    and not relative to boat'''
def ADCP_callb(msg):
    global current, ADCP_mean
    current = msg.data
    calc_mean = rospy.get_param('ADCP/mean', False)
    if calc_mean:
        if rospy.get_param('/ADCP/reset', False):
            ADCP_mean = [0.0, 0.0, 0.0]
            rospy.set_param('/ADCP/reset', False)

        ADCP_mean[0] += current[1]
        ADCP_mean[1] += 1
        ADCP_mean[2] = ADCP_mean[0]/ADCP_mean[1]


def lidar_callb(msg):
    global ranges
    ranges = msg.ranges



def navGoal_callb(msg):
    global wayPoints
    point = msg.pose.position
    state_ref[0] = point.x
    state_ref[1] = point.y
    state_ref[2] = point.z
    #print("WP added! ", point)
    #print("state ref:", state_ref)
    #print(wayPoints)

def switchControl():
    controller_type = rospy.get_param('/nav_mode', 'Waypoint')
    if (controller_type == "Waypoint"):
        return PI_controller
    elif (controller_type == "Transect"):
        return transect_controller
    else:
        return PI_controller


# STATE:
# START - goto start point
# HOLD - keep position, measure mean current angle and calculate transect
# TRANSECT1 - first transect, measure mean current angle
# CALCULATE - calc new transect based on current mean
# TRANSECT2 - transects
# HOME - go home

ctrl_pub = rospy.Publisher('motor_controller/motor_cmd_reciever', MotorCommand, queue_size=1)
state_pub = rospy.Publisher('controller/STATE', String, queue_size=1)
STATE = 'none'
start_time = 0.0
direction = True #true - look at shore to the right, false - look at shore to the left
transect_cnt = 0
home_coord = []

def main():
    global samp_time, wayPoints, h, current, STATE, home_coord
    rospy.init_node('master_controller')

    # information update subscriber
    rospy.Subscriber('GPS/xy_coord', GPS_data, GPS_callb)
    rospy.Subscriber('heading', Float32, IMU_callb)
    rospy.Subscriber('ControlCenter/gps_wp', GPS_WayPoints, WP_callb)
    rospy.Subscriber('adcp/data', Float32MultiArray, ADCP_callb) # needs fixing for real thing
    rospy.Subscriber('move_base_simple/goal', PoseStamped, navGoal_callb)

    rate = rospy.Rate(1/h)

    STATE = START
    home_coord = [state_asv[0], state_asv[1]]

    while not rospy.is_shutdown():
        if STATE == 'START':
            start()
        elif STATE == 'HOLD':
            hold()
        elif STATE == 'TRANSECT':
            transect()
        elif STATE == 'HOME':
            home()
        else:
            print('not a valid state')
            #fix so ros error message

        rate.sleep()


def destinationReached():
    global state_asv, state_ref
    DIST_THRESHOLD = rospy.get_param('/dist_threshold', 1.0)
    dist = math.sqrt((state_ref[0] - state_asv[0])**2 + (state_ref[1] - state_asv[1])**2)
    return dist < DIST_THRESHOLD

#returns distance to specified angle (global frame), returns mean value of specified number of points
def get_distance(ang, nbr_of_points=9):
    global state_asv, ranges
    nbr = math.floor(nbr_of_points/2)
    inc = 2*math.pi/len(ranges)
    index = int((state_asv[2] - ang + math.pi)/inc)
    sum = ranges[index]
    for i in range(1,nbr+1):
        sum += math.cos(inc*i)*(ranges[index+i] + ranges[index-i])

    # return mean of nbr_of_points (uneven) closest points
    return sum/(2*nbr+1)

# Compares distance to shore on either right or left side to the threshold.
# If the number of points exceeds some number, turn around
def to_close(dir):
    global state_asv, wayPoints, ranges
    inc = 2*math.pi/len(ranges)
    dist_th = rospy.get_param('dist_th')
    theta_p = np.arctan2(wayPoints[0].y - wayPoints[1].y, wayPoints[0].x - wayPoints[1].x)

    #Look at an angle of pi/4 above and below transect point
    if dir:
        start_ang = angleDiff(theta_p - state_asv[2] - math.pi/4)
        end_ang = angleDiff(theta_p - state_asv[2] + math.pi/4)
    elif not dir:
        start_ang = angleDiff(theta_p - state_asv[2] + math.pi - math.pi/4)
        end_ang = angleDiff(theta_p - state_asv[2] + math.pi + math.pi/4)

    #If more than a specified number of the points in the region specified above are within threshold, turn around
    dists = ranges[np.arange(int((start_ang + math.pi)/inc) % len(ranges), int((end_ang + math.pi)/inc) % len(ranges))]
    close = np.nonzero(dists < dist_th)
    return len(close) > 10 #is this a reasonable way of doing it? now turns if more than 10 values are to close...

#Get shortest distance from lidar in specified interval
def get_shortest(start_ang, end_ang):
    global ranges
    inc = 2*math.pi/len(ranges)
    dists = ranges[np.arange(int((start_ang + math.pi)/inc) % len(ranges), int((end_ang + math.pi)/inc) % len(ranges))]
    return np.amin(dists)

def publish_cmds(controller):
    global  ctrl_pub
    motor_cmd = MotorCommand()

    u_thrust, u_rudder = controller.calc_control()
    motor_cmd.port = u_thrust
    motor_cmd.strboard = u_thrust
    motor_cmd.servo = u_rudder

    rospy.logdebug('MotorCmd ' + str(motor_cmd))
    ctrl_pub.publish(motor_cmd)

#Calculates two transect points (on land) creating a line perpendicular to current
def calculate_transect(theta_c):
    global state_asv
    point1 = GPS_data()
    point2 = GPS_data()
    distL = get_distance(theta_c + math.pi/2, 21)
    distR = get_distance(theta_c - math.pi/2, 21)
    point1.x = state_asv[0] + (distR + 10) * math.cos(theta_c - math.pi/2)
    point1.y = state_asv[1] + (distR + 10) * math.sin(theta_c - math.pi/2)
    point2.x = state_asv[0] + (distL + 10) * math.cos(theta_c + math.pi/2)
    point2.y = state_asv[1] + (distL + 10) * math.sin(theta_c + math.pi/2)

    #how should the points be saved for transect controller??
    return [point1, point2]


#add destReached to update variable ?
#navigates to user specified start point, when there start ADCP mean calculations
def start():
    global start_time, state_asv, state_ref, v_asv, target_index, wayPoints, current
    state_pub.publish(STATE)
    state_ref[0] = rospy.get_param('/start_x')
    state_ref[1] = rospy.get_param('/start_y')
    destReached = destinationReached()
    PI_controller.update_variable(state_asv, state_ref, v_asv, target_index, wayPoints, current)
    PI_controller.destReached(destReached)
    publish_cmds(PI_controller)
    if destReached:
        STATE = 'HOLD'
        start_time = rospy.get_rostime()
        rospy.set_param('/ADCP/mean', True)
        rospy.set_param('/ADCP/reset', True)

#Hold position for specified time, to measure mean angle of current, after specified time
#calculate transect based on it
def hold():
    global start_time, state_asv, state_ref, v_asv, target_index, wayPoints, current, ADCP_mean, direction
    state_pub.publish(STATE)
    waitTime = rospy.get_param('/wait_time', 5.0)
    PI_controller.update_variable(state_asv, state_ref, v_asv, target_index, wayPoints, current)
    PI_controller.destReached(True)
    publish_cmds(PI_controller)
    if rospy.get_rostime - start_time > waitTime:
        wayPoints = calculate_transect(ADCP_mean[2])

        STATE = 'TRANSECT'
        direction = True
        rospy.set_param('/ADCP/reset', True)

#Make transect, after first completed transect (back and forth) calculate new transect based on new mean current
#continue for specified time or number of transects
def transect():
    global state_asv, state_ref, v_asv, target_index, wayPoints, current, ADCP_mean, direction
    state_pub.publish(STATE)
    run_time = rospy.get_param('/run_time', inf)
    max_transect = rospy.get_param('/max_runtime', inf)
    state_pub.publish(STATE)
    if to_close(direction):
        transect_cnt += 1
        direction = not direction

    if transect_cnt == 2:
        wayPoints = calculate_transect(ADCP_mean[2])

    if rospy.get_rostime() - start_time > run_time or transect_cnt > max_transect:
        STATE = 'HOME'
    else:
        controller
        # calculate/actuate transect

#go back to home coordinates
#keep constant distance to shore on the way back
#when close enough, go to home point
def home():
    global state_asv, state_ref, v_asv, target_index, wayPoints, current, ADCP_mean, direction, home_coord
    state_pub.publish(STATE)
    dist_shore = rospy.get_param('/dist_shore', 5.0)
    theta_h = angleDiff(math.atan2(home_coord[1] - state_asv[1], home_coord[0] - state_asv[0]))
    #if dir_home = 'down':
    if (math.pi/4 <= abs(theta_h) <= 3*math.pi/4) and ((home_coord[0] - state_asv[0])**2 + (home_coord[1] - state_asv[1])**2) > 2*dist_shore:
    #measure distance to shore, keep desired distance
        #how to keep track of orientation of shore?? Now assumes it is parallel to current
        ang = current[1] - 2*math.pi/3 #if downstream
        distance = get_distance(ang, 21)
        '''transform nav_points from robot's to global coordinate system'''
        rot = np.array([[np.cos(state_asv[2]), -np.sin(state_asv[2])], \
            [np.sin(state_asv[2]), np.cos(state_asv[2])]])
        navPoint_robot = np.array([distance * math.cos(state_asv[2] - ang), \
            [np.sign(math.sin(ang)) * (distance * abs(math.sin(ang)) - dist_shore)]])
        navPoint_global = np.matmul(rot, navPoint_robot)
        state_ref = [state_asv[0] + navPoint_global[0], state_asv[1] + navPoint_global[1]]
        PI_controller.destReached(False)
    else:
        state_ref = [home_coord[0], home_coord[1]]
        PI_controller.destReached(destinationReached())
        # + set v_ref pretty low

    PI_controller.update_variable(state_asv, state_ref, v_asv, target_index, wayPoints, current)
    publish_cmds(PI_controller)
