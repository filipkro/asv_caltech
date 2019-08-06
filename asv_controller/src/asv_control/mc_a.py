#!/usr/bin/env python

import rospy
import PI_controller
import transect_controller
import math
import tf
import numpy as np
from gps_reader.msg import GPS_data, GPS_WayPoints
from motor_control.msg import MotorCommand
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Float32, Float32MultiArray, String, Int64MultiArray
from sensor_msgs.msg import LaserScan
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
STATE = ''
lidar_inc = 0.0
transect_cnt = 0 # number of transects made

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

    trans_broadcast()

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
    else:
        ADCP_mean[2] = v_angle

def ADCP_callb_sim(msg):
    global current, ADCP_mean
    current = msg.data
    calc_mean = rospy.get_param('ADCP/mean', False)
    if calc_mean:
        if rospy.get_param('/ADCP/reset', False):
            ADCP_mean = [0.0, 0.0, 0.0] # [sum of average, num_samples, mean]
            rospy.set_param('/ADCP/reset', False)

        ADCP_mean[0] += current[1]
        ADCP_mean[1] += 1
        ADCP_mean[2] = ADCP_mean[0]/ADCP_mean[1]
    else:
        ADCP_mean[2] = current[1]

def s16(value):
    ''' convert unsigned integer to signed integer'''
    return -(value & 0x8000) | (value & 0x7fff)


def lidar_callb(msg):
    global ranges, lidar_inc
    ranges = np.array(msg.ranges)
    print(len(ranges))
    lidar_inc = 2*math.pi/len(ranges)
    print(lidar_inc)


def updateTarget():
    '''Update target way point based on current position
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
start_time = 0.0
direction = False #true - look at shore to the right, false - look at shore to the left
transect_cnt = 0
home_coord = []

def main():
    global wayPoints, h, current, STATE, home_coord
    rospy.init_node('master_controller')

    # information update subscriber
    rospy.Subscriber('GPS/xy_coord', GPS_data, GPS_callb)
    rospy.Subscriber('heading', Float32, IMU_callb)
    rospy.Subscriber('ControlCenter/gps_wp', GPS_WayPoints, WP_callb)
    # rospy.Subscriber('adcp/data', Int64MultiArray, ADCP_callb) # needs fixing for real thing
    rospy.Subscriber('adcp/data', Float32MultiArray, ADCP_callb_sim)
    rospy.Subscriber('move_base_simple/goal', PoseStamped, navGoal_callb)
    rospy.Subscriber('/os1/scan', LaserScan, lidar_callb)

    rate = rospy.Rate(1/h)

    STATE = 'START'
    home_coord = [state_asv[0], state_asv[1]]


    while not rospy.is_shutdown():
        print(STATE)
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


def get_distance(ang, nbr_of_points=5):
    '''Getting the mean distance to the specified angle
        ang: angle to look for shortest range
        nbr_of_points: number of readings to average over'''
    global state_asv, ranges, lidar_inc
    nbr = int(math.floor(nbr_of_points/2))
    inc = lidar_inc
    print(inc)
    index = int((angleDiff(state_asv[2] - ang) + math.pi)/inc)
    range_sum = ranges[index]
    for i in range(1,nbr+1):
        range_sum += math.cos(inc*i)*(ranges[(index+i) % len(ranges)] + ranges[(index-i) % len(ranges)])

    # return mean of nbr_of_points (uneven) closest points
    return range_sum/(2*nbr+1)

# Compares distance to shore on either right or left side to the threshold.
# If the number of points exceeds some number, turn around
def too_close(dir):
    global state_asv, wayPoints, ranges, lidar_inc
    inc = lidar_inc
    dist_th = rospy.get_param('dist_th', 5.0)
    theta_p = np.arctan2(wayPoints[0].y - wayPoints[1].y, wayPoints[0].x - wayPoints[1].x)

    #Look at an angle of pi/4 above and below transect point
    if dir:
        start_ang = angleDiff(theta_p - math.pi/4)
        end_ang = angleDiff(theta_p  + math.pi/4)
    elif not dir:
        start_ang = angleDiff(theta_p  + math.pi - math.pi/4)
        end_ang = angleDiff(theta_p  + math.pi + math.pi/4)

    #If more than a specified number of the points in the region specified above are within threshold, turn around
    if (int((start_ang + math.pi)/inc) <  int((end_ang + math.pi)/inc)): 
        dists = ranges[np.arange(int((start_ang + math.pi)/inc) % len(ranges), int((end_ang + math.pi)/inc) % len(ranges))]
    else:
        dists1 = ranges[np.arange(int((start_ang + math.pi)/inc), len(ranges))]
        dists2 = ranges[np.arange(0, int((end_ang + math.pi)/inc))]
        dists = np.concatenate((dists1, dists2))
        print(dists)
    
    close = np.nonzero(dists < dist_th)
    
    print(close[0])
    print('length of close ' , len(close[0]))
    return len(close[0]) > 10 #is this a reasonable way of doing it? now turns if more than 10 values are to close...

#Get shortest distance from lidar in specified interval
def get_shortest(start_ang, end_ang):
    global ranges, lidar_inc
    inc = lidar_inc
    dists = ranges[np.arange(int((angleDiff(start_ang) + math.pi)/inc) % len(ranges), int((angleDiff(end_ang) + math.pi)/inc) % len(ranges))]
    return np.amin(dists)

def publish_cmds(controller):
    global  ctrl_pub
    motor_cmd = MotorCommand()

    u_thrust, u_rudder = controller.calc_control()
    motor_cmd.port = u_thrust
    motor_cmd.strboard = u_thrust
    motor_cmd.servo = u_rudder

    rospy.logdebug('MotorCmd ' + str(motor_cmd))
    print(motor_cmd)
    ctrl_pub.publish(motor_cmd)

def trans_broadcast():
    global state_asv
    br = tf.TransformBroadcaster()
    br.sendTransform((state_asv[0], state_asv[1], 0),
                tf.transformations.quaternion_from_euler(0, 0, state_asv[2]),
                rospy.Time.now(),
                'os1_lidar',
                "map")


#Calculates two transect points (on land) creating a line perpendicular to current
def calculate_transect(theta_c):
    '''Input:
         theta_c: current angle (float) in global frame
       Output:
         [state, state]: two points on the line normal to theta_c '''
    global state_asv
    point1 = GPS_data()
    point2 = GPS_data()
    # sample cerain number of points from the sides of the current angle
    distL = get_distance(theta_c + math.pi/2, 21)
    print('distL', distL)
    distR = get_distance(theta_c - math.pi/2, 21)
    print('distR', distR)

    # generate points using simple trig
    point1.x = state_asv[0] + (distR + 10) * math.cos(theta_c - math.pi/2)
    point1.y = state_asv[1] + (distR + 10) * math.sin(theta_c - math.pi/2)
    point2.x = state_asv[0] + (distL + 10) * math.cos(theta_c + math.pi/2)
    point2.y = state_asv[1] + (distL + 10) * math.sin(theta_c + math.pi/2)

    print(point1)
    print(point2)

    #how should the points be saved for transect controller??
    return [point1, point2]


#add destReached to update variable ?
#navigates to user specified start point, when there start ADCP mean calculations
def start():
    '''Simple navigation to the first destination point'''
    global start_time, state_asv, state_ref, v_asv, \
            STATE, target_index, wayPoints, current
    state_pub.publish(STATE)
    state_ref[0] = rospy.get_param('/start_x', 0.0)
    state_ref[1] = rospy.get_param('/start_y', 5.0)
    destReached = destinationReached()
    PI_controller.update_variable(state_asv, state_ref, v_asv, target_index, wayPoints, current)
    PI_controller.destinationReached(destReached)
    publish_cmds(PI_controller)
    if destReached:
        STATE = 'HOLD'
        start_time = rospy.get_rostime()
        rospy.set_param('/ADCP/reset', True)
        rospy.set_param('/ADCP/mean', True)

#Hold position for specified time, to measure mean angle of current, after specified time
#calculate transect based on it
def hold():
    '''Holding at the line to get a reliable reading on current angle'''
    global start_time, state_asv, state_ref, v_asv, target_index, wayPoints, \
            STATE, current, ADCP_mean, direction, ranges
    state_pub.publish(STATE)
    waitTime = rospy.get_param('/wait_time', 5.0)
    state_ref = state_asv
    PI_controller.update_variable(state_asv, state_ref, v_asv, target_index, wayPoints, current)
    PI_controller.destinationReached(True)
    publish_cmds(PI_controller)

    if (rospy.get_rostime() - start_time).to_sec() > waitTime: # and len(ranges) != 0:
        #calculate two points on the line normal to the water current
        wayPoints = calculate_transect(ADCP_mean[2])
        print('wP: ', wayPoints)
        print(len(ranges))

        STATE = 'TRANSECT'
        # STATE = 'HOLD' # just for testing purposes
        direction = True
        rospy.set_param('/ADCP/reset', True)

#Make transect, after first completed transect (back and forth) calculate new transect based on new mean current
#continue for specified time or number of transects
def transect():
    global state_asv, state_ref, v_asv, target_index, wayPoints, current, \
        STATE, ADCP_mean, direction, transect_cnt
    state_pub.publish(STATE)
    run_time = rospy.get_param('/run_time', 100)
    max_transect = rospy.get_param('/max_runtime', 10)
    state_pub.publish(STATE)
    if too_close(direction):
        print('too close')
        transect_cnt += 1
        direction = not direction

    if transect_cnt == 2:
        wayPoints = calculate_transect(ADCP_mean[2])

    if (rospy.get_rostime() - start_time).to_sec() > run_time or transect_cnt > max_transect:
        STATE = 'HOME'
    else:
        controller = transect_controller
        if (direction):
            target_index = 1
        else:
            target_index = 0

        print('target index', target_index)

        transect_controller.update_variable(state_asv, state_ref, v_asv,
                            target_index, wayPoints, current)
        publish_cmds(transect_controller)
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
        navPoint_robot = np.array([[distance * math.cos(state_asv[2] - ang)], \
            [np.sign(math.sin(ang)) * (distance * abs(math.sin(ang)) - dist_shore)]])

        navPoint_global = np.matmul(rot, navPoint_robot)
        state_ref = [state_asv[0] + navPoint_global[0], state_asv[1] + navPoint_global[1]]
        PI_controller.destinationReached(False)
    else:
        state_ref = [home_coord[0], home_coord[1]]
        PI_controller.destinationReached(destinationReached())
        # + set v_ref pretty low

    PI_controller.update_variable(state_asv, state_ref, v_asv, target_index, wayPoints, current)
    publish_cmds(PI_controller)

def angleDiff(angle):
    while(angle > math.pi):
        angle = angle - math.pi * 2
    while (angle < -math.pi):
        angle = angle + math.pi * 2
    return angle

## for quick test purposes, remove when use this in a node
if __name__ == "__main__":
    main()
