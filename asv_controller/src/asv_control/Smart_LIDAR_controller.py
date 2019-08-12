#!/usr/bin/env python

#state.py
#using this guy's design
# https://dev.to/karn/building-a-simple-state-machine-in-python
import rospy
from state import State # state machine
import PI_controller
import transect_controller
import math
import tf
import numpy as np
from Generic_Controller import Generic_Controller
from gps_reader.msg import GPS_data, GPS_WayPoints
from motor_control.msg import MotorCommand
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Float32, Float32MultiArray, String, Int64MultiArray
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray # for simulation


########## Smart LiDAR controller state machine #######
class Smart_LiDAR_Controller(Generic_Controller):
    def __init__(self):
        Generic_Controller.__init__(self)
        self.state = Start()
        self.ranges = []
        self.lidar_inc = 0.0

    def update_lidar(self, ranges, inc):
        '''get lidar readings'''
        self.ranges = ranges
        self.lidar_inc = 2*math.pi/400
        print('lidar_inc (in update_lidar) ', self.lidar_inc)

    def calc_control(self):
        # Update the controller
        # Important some controllers might use their own info, such as Transect
        self.state.update_controller_var(self.state_asv, self.state_ref, \
                self.v_asv, self.target_index, self.wayPoints, self.current)
        self.state.controller.destinationReached(self.destReached)
        # state transition
        self.state = self.state.on_event('Run')
        self.state.update_lidar(self.ranges, self.lidar_inc)
        print('STATE: ', self.state)
        return self.state.calc_control()

########## Specific states ############

class Start(State):
    ''' This initial state of the controller drives the robot to a designated
        point '''
    def __init__(self):
        State.__init__(self)
        self.controller = PI_controller.PI_controller()
        self.xref = rospy.get_param('/start/x', 0.0)
        self.yref = rospy.get_param('/start/y', 2.0)

    def on_event(self, event):
        '''
        Input:
            event: (str) a string that describes an event happend in the asv
        Output:
            state: (State) the next state
        '''

        state_asv = self.controller.state_asv
        state_ref = self.controller.state_ref
        DIST_THRESHOLD = rospy.get_param('/dist_threshold', 1.0)
        dist = math.sqrt((state_ref[0] - state_asv[0])**2 + (state_ref[1] - \
                         state_asv[1])**2)

        if dist < DIST_THRESHOLD:
            return Hold()
        else:
            return self

    def calc_control(self):
        '''remember to update the controller before calling this'''
        self.controller.state_ref[0] = self.xref
        self.controller.state_ref[1] = self.yref
        return self.controller.calc_control()


class Hold(State):
    '''Hold at the point until the asv figures out the current'''
    def __init__(self):
        State.__init__(self)
        self.controller = PI_controller.PI_controller()
        self.start_time = rospy.get_rostime()
        self.waitTime = rospy.get_param('/hold/wait_time', 5.0)

    def on_event(self, event):
        self.waitTime = rospy.get_param('/hold/wait_time', 5.0)
        if (rospy.get_rostime() - self.start_time).to_sec() > self.waitTime:
            return Transect(last_controller=self.controller, ranges=self.ranges,
                                lidar_inc=self.lidar_inc)
        else:
            return self

    def calc_control(self):
        '''remember to update the controller before calling this'''
        self.controller.destinationReached(True)
        # self.controller.state_ref = self.controller.state_asv
        return self.controller.calc_control()

class Transect(State):
    '''Perform transect on the spot'''

    def __init__(self, last_controller=None, ranges=None, lidar_inc=None, transect = True, dir=False, trans_cnt=None):
        State.__init__(self)
        self.controller = transect_controller.Transect_controller(controller=last_controller)
        if trans_cnt == None:
            self.transect_cnt = 0
            self.start_time = rospy.get_rostime()
        else:
            self.transect_cnt = trans_cnt[0]
            self.start_time = trans_cnt[1]
        self.run_time = rospy.get_param('/transect/run_time', 300.0)
        self.max_transect = rospy.get_param('/transect/max_transect', 5)
        self.dist_th = rospy.get_param('/transect/dist_threshold', 3.0)
        
        self.transect_time_ref = 5.0
        self.direction = dir #False - look at shore to the left, True-look at shore to the right
        print('direction (in transect) ', self.direction)
        self.p1 = GPS_data()
        self.p2 = GPS_data()


        self.ranges = ranges
        self.lidar_inc = lidar_inc

        # calculate transect once upon initialization
        current = self.controller.current # fix the averaging
        current_angle = current[1]

        if transect:
            [self.p1, self.p2] = self.calculate_transect(current_angle)
            # IMPORTANT: Transect state disregard target_index, wayPoints information
            # from the top controller. It instead has its own
            if self.direction:
                self.target_index = 0
            else:
                self.target_index = 1
            self.wayPoints = [self.p1, self.p2]

    def on_event(self, event):
        self.run_time = rospy.get_param('/transect/run_time', 100.0)
        self.max_transect = rospy.get_param('/transect/max_transect', 5)
        self.upstream = rospy.get_param('/upstream', True)
        print('direction (in on_event) ', self.direction)
        if (rospy.get_rostime() - self.start_time).to_sec() > self.run_time:
            return Home(self.ranges, self.controller.state_asv, self.controller.current)
        elif self.too_close(self.direction):
            print('?')
            self.direction = not self.direction
            self.transect_cnt += 1
            if self.transect_cnt > self.max_transect:
                return Home(self.ranges, self.controller.state_asv, self.controller.current)
            if (self.direction):
                self.target_index = 0
            else:
                self.target_index = 1
            if self.upstream:
                trans_cnt = [self.transect_cnt, self.start_time]
                return Upstream(self.ranges, self.controller.state_asv, self.controller.current, self.direction, trans_cnt)
            else:
                return self
        else:
            return self


    def calc_control(self):
        '''remember to update the controller before calling this'''
        #print('controller wp' , self.controller.wayPoints)
        print('wayPoints', self.wayPoints)
        self.controller.wayPoints = self.wayPoints
        self.controller.target_index = self.target_index
        print(self.controller.target_index)
        print(self.direction)
        return self.controller.calc_control()

    def get_distance(self, ang, nbr_of_points=5):
        '''Getting the mean distance to the specified angle
            ang: angle to look for shortest range
            nbr_of_points: number of readings to average over'''
        state_asv = self.controller.state_asv
        ranges = self.ranges
        lidar_inc = self.lidar_inc
        print('ranges[350]', ranges[350])
        nbr = int(math.floor(nbr_of_points/2))
        inc = lidar_inc
        print('angle to wall ', ang)
        print('theta ', state_asv[2])
        index = int((self.controller.angleDiff(ang - state_asv[2]) \
                                                    + math.pi)/inc) % len(ranges)
        print('index ', index)
        range_sum = ranges[index]
        for i in range(1,nbr+1):
            range_sum += math.cos(inc*i)*(ranges[(index+i) % len(ranges)] \
                            + ranges[(index-i) % len(ranges)])

        # return mean of nbr_of_points (uneven) closest points
        return range_sum/(2*nbr+1)

    #Calculates two transect points (on land) creating a line perpendicular to current
    def calculate_transect(self, theta_c):
        '''Input:
             theta_c: current angle (float) in global frame
           Output:
             [state, state]: two points on the line normal to theta_c '''
        state_asv = self.controller.state_asv
        point1 = GPS_data()
        point2 = GPS_data()

        # sample cerain number of points from the sides of the current angle
        distL = self.get_distance(theta_c + math.pi/2, 21)
        distR = self.get_distance(theta_c - math.pi/2, 21)

        # generate points using simple trig
        point1.x = state_asv[0] + (distR + 10) * math.cos(theta_c - math.pi/2)
        point1.y = state_asv[1] + (distR + 10) * math.sin(theta_c - math.pi/2)
        point2.x = state_asv[0] + (distL + 10) * math.cos(theta_c + math.pi/2)
        point2.y = state_asv[1] + (distL + 10) * math.sin(theta_c + math.pi/2)


        #how should the points be saved for transect controller??
        return [point1, point2]

    def too_close(self, dir):
        ''' Compares distance to shore on either right or left side to the threshold.
            If the number of points exceeds some number, turn around '''
        state_asv = self.controller.state_asv
        ranges = self.ranges
        inc = self.lidar_inc
        wayPoints = self.controller.wayPoints

        self.dist_th = rospy.get_param('/transect/dist_threshold', 3.0)
        dist_th = self.dist_th

        theta_p = self.controller.angleDiff(np.arctan2(wayPoints[0].y \
                            - wayPoints[1].y, wayPoints[0].x - wayPoints[1].x))

        #Look at an angle of pi/4 above and below transect point
        if dir:
            start_ang = self.controller.angleDiff(theta_p - \
                                            state_asv[2] - math.pi/4 + math.pi)
            end_ang = self.controller.angleDiff(theta_p - \
                                            state_asv[2] + math.pi/4 + math.pi)
        elif not dir:
            start_ang = self.controller.angleDiff(theta_p  \
                                                   - state_asv[2]  - math.pi/4)
            end_ang = self.controller.angleDiff(theta_p - \
                                                       state_asv[2]  + math.pi/4)


        #print('start_indx: ', int((start_ang + math.pi)/inc))
        #print('end_indx: ', int((end_ang + math.pi)/inc))
        #If more than a specified number of the points in the region specified above are within threshold, turn around
        if (int((start_ang + math.pi)/inc) <  int((end_ang + math.pi)/inc)):
            dists = ranges[np.arange(int((start_ang + math.pi)/inc) % len(ranges), int((end_ang + math.pi)/inc) % len(ranges))]
        else:
            dists1 = ranges[np.arange(int((start_ang + math.pi)/inc), len(ranges))]
            dists2 = ranges[np.arange(0, int((end_ang + math.pi)/inc))]
            dists = np.concatenate((dists1, dists2))
        close = np.nonzero(dists < dist_th)

        return len(close[0]) > 10 #is this a reasonable way of doing it? now turns if more than 10 values are to close...

# class Middle(State):
#     '''Move boat to middle of the river before going upstream'''
#     def __init__(self, last_direction):
#         State.__init__(self)
#         self.controller = transect_controller.Transect_controller()
#         dist_calc = Transect(last_controller=self.controller, ranges=self.ranges, \
#                             lidar_inc=self.lidar_inc)
#
#         if last_direction: #right?
#             theta_p = self.controller.angleDiff(self.controller.current[1] - math.pi/2) # use average instead
#         else: #left? directions might be swapped
#             theta_p = self.controller.angleDiff(self.controller.current[1] - math.pi/2) # use average instead
#         dist = dist_calc.get_distance(theta_p)
#         #approximately mid point of previous transect
#         self.controller.state_ref[0] = self.controller.state_asv[0] + dist * math.cos(theta_p)/2
#         self.controller.state_ref[1] = self.controller.state_asv[1] + dist * math.sin(theta_p)/2
#
#     def on_event(self, event):
#         dist = math.sqrt((self.controller.state_ref[0] - self.controller.state_asv[0])**2 + \
#                         (self.controller.state_ref[1] - self.controller.state_asv[1])**2)
#         DIST_THRESHOLD = rospy.get_param('/dist_threshold', 1.0)
#         if dist < DIST_THRESHOLD:
#             return Upstream()
#         elif event == 'go_home':
#             return Home()
#         else:
#             return self
#
#     def calc_control(self):
#         '''remember to update the controller before calling this'''
#         #print('controller wp' , self.controller.wayPoints)
#         return self.controller.calc_control()
#
# class Upstream(State):
#     '''Move upstream against the current'''
#     def __init__(self):
#         State.__init__(self)
#         self.controller = PI_controller.PI_controller()
#         dist_upstream = rospy.get_param('/dist_upstream', 5.0)
#         self.controller.state_ref[0] = self.controller.state_asv[0] + \
#                     dist_upstream * math.cos(self.controller.current[1]) # use average current instead
#         self.controller.state_ref[1] = self.controller.state_asv[1] + \
#                     dist_upstream * math.sin(self.controller.current[1]) # use average current instead
#
#
#     def on_event(self, event):
#         dist = math.sqrt((self.controller.state_ref[0] - self.controller.state_asv[0])**2 + \
#                         (self.controller.state_ref[1] - self.controller.state_asv[1])**2)
#         DIST_THRESHOLD = rospy.get_param('/dist_threshold', 1.0)
#         if dist < DIST_THRESHOLD:
#             return Transect()
#         elif event == 'go_home':
#             return Home()
#         else:
#             return self
#
#     def calc_control(self):
#         '''remember to update the controller before calling this'''
#         #print('controller wp' , self.controller.wayPoints)
#         return self.controller.calc_control()

class Upstream(State):
    '''Move upstream to point in the middle a specified distance away, then do new transect there'''
    def __init__(self, lidar_readings, last_state, current, dir, trans_cnt):
        State.__init__(self)
        self.ranges = lidar_readings
        self.controller = PI_controller.PI_controller()
        self.controller.state_asv = last_state
        self.controller.current = current
        self.direction = dir
        self.trans_cnt = trans_cnt
        print('direction ', dir)
        # self.dist_calc = Transect(last_controller=self.controller, ranges=self.ranges, \
                            # lidar_inc=self.lidar_inc, transect=False)
        self.update_ref()

    def update_ref(self):
        '''Update reference point, middle of river a specified distance downstream'''
        # self.dist_calc.ranges = self.ranges
        # self.dist_calc.controller.state_asv = self.controller.state_asv
        # print('current ', self.controller.current[1])
        # d_left = self.dist_calc.get_distance(self.controller.current[1] + math.pi/2) #use average instead
        # print('dLeft ', d_left)
        # d_right = self.dist_calc.get_distance(self.controller.current[1] - math.pi/2) #use average instead
        print('current ', self.controller.current[1])
        d_left = self.get_distance(self.controller.current[1] + math.pi/2) #use average instead
        print('dLeft ', d_left)
        d_right = self.get_distance(self.controller.current[1] - math.pi/2) #use average instead
        print('dRight ', d_right)
        dist_mid = (d_right - d_left)/4 #/2 for middle
        print('dist_mid ', dist_mid)
        theta_p = self.controller.current[1] - np.sign(dist_mid) * math.pi/2
        print('theta_p ', theta_p)
        dist_upstream = np.sign(math.sin(self.controller.current[1])) * rospy.get_param('/dist_upstream', 5.0)
        print('dist_upstream ', dist_upstream)
        theta_dest = self.controller.angleDiff(theta_p + np.sign(dist_upstream*dist_mid)*(math.atan2(abs(dist_upstream), abs(dist_mid))))
        print('theta_dest ', theta_dest)
        dist = math.sqrt(dist_mid**2 + dist_upstream**2)
        print('dist ', dist)
        self.xref = self.controller.state_asv[0] + dist * math.cos(theta_dest)
        self.yref = self.controller.state_asv[1] + dist * math.sin(theta_dest)

    def get_distance(self, ang, nbr_of_points=5):
        '''Getting the mean distance to the specified angle
            ang: angle to look for shortest range
            nbr_of_points: number of readings to average over'''
        state_asv = self.controller.state_asv
        ranges = self.ranges
        lidar_inc = 2*math.pi / len(ranges)
        print('inc', lidar_inc)
        print('ranges[350]', ranges[350])
        nbr = int(math.floor(nbr_of_points/2))
        print('angle to wall ', ang)
        print('theta ', state_asv[2])
        index = int((self.controller.angleDiff(ang - state_asv[2]) \
                                                    + math.pi)/lidar_inc) % len(ranges)
        print('angleDiff ', self.controller.angleDiff(ang - state_asv[2]))
        print('everything excpt mod', int((self.controller.angleDiff(ang - state_asv[2]) + math.pi)/lidar_inc))
        print('index ', index)
        range_sum = ranges[index]
        for i in range(1,nbr+1):
            range_sum += math.cos(lidar_inc*i)*(ranges[(index+i) % len(ranges)] \
                            + ranges[(index-i) % len(ranges)])

        # return mean of nbr_of_points (uneven) closest points
        return range_sum/(2*nbr+1)

    def on_event(self, event):
        dist = math.sqrt((self.controller.state_ref[0] - self.controller.state_asv[0])**2 + \
                        (self.controller.state_ref[1] - self.controller.state_asv[1])**2)
        DIST_THRESHOLD = rospy.get_param('/dist_threshold', 1.0)
        run_time = rospy.get_param('/transect/run_time', 100.0)
        if (rospy.get_rostime() - self.trans_cnt[1]).to_sec() > run_time:
            return Home(self.ranges, self.controller.state_asv, self.controller.current)
        if dist < DIST_THRESHOLD:
            print('direction ', self.direction)
            return Transect(last_controller=self.controller, ranges=self.ranges,
                                lidar_inc=self.lidar_inc, dir=self.direction, trans_cnt=self.trans_cnt)
        else:
            return self

    def calc_control(self):
        '''remember to update the controller before calling this'''
        #print('controller wp' , self.controller.wayPoints)
        self.controller.state_ref[0] = self.xref
        self.controller.state_ref[1] = self.yref
        return self.controller.calc_control()

class Home(State):
    '''Go back to hem (home)! Hem sweet hem (home ljuva home).'''
    def __init__(self, lidar_readings, last_state, current):
        State.__init__(self)
        self.ranges = lidar_readings
        self.controller = PI_controller.PI_controller()
        self.controller.state_asv = last_state
        self.controller.current = current
        self.dist_calc = Transect(last_controller=self.controller, ranges=self.ranges, \
                            lidar_inc=self.lidar_inc, transect=False)
        self.home = [rospy.get_param('/home/x', 0.0), rospy.get_param('/home/y', 0.0)]
        self.update_ref()

    def update_ref(self, go_home=False):
        '''Update reference point, middle of river a specified distance downstream'''
        if go_home:
            self.xref = self.home[0]
            self.yref = self.home[1]
        else:
            self.dist_calc.ranges = self.ranges
            # d_left = self.dist_calc.get_distance(self.controller.current[1] + math.pi/2) #use average instead
            # d_right = self.dist_calc.get_distance(self.controller.current[1] - math.pi/2) #use average instead
            d_left = self.get_distance(self.controller.current[1] + math.pi/2) #use average instead
            d_right = self.get_distance(self.controller.current[1] - math.pi/2) #use average instead
            dist_mid = (d_right - d_left)/4 # /2 for middle
            theta_p = self.controller.current[1] - np.sign(dist_mid) * math.pi/2
            dist_downstream = np.sign(math.sin(self.controller.current[1])) * rospy.get_param('/dist_downstream', 5.0)
            theta_dest = self.controller.angleDiff(theta_p - np.sign(dist_downstream*dist_mid)* math.atan2(abs(dist_downstream), abs(dist_mid)))
            dist = math.sqrt(dist_mid**2 + dist_downstream**2)
            self.xref = self.controller.state_asv[0] + dist * math.cos(theta_dest)
            self.yref = self.controller.state_asv[1] + dist * math.sin(theta_dest)


    def on_event(self, event):
        theta_home = self.controller.state_asv[2] - math.atan2(self.home[1] - self.controller.state_asv[1], \
                            self.home[0] - self.controller.state_asv[0])
        if abs(theta_home) <= 2*math.pi/3:
            go_home = True
        else:
            go_home = False
        self.update_ref(go_home)
        dist = math.sqrt((self.controller.state_ref[0] - self.controller.state_asv[0])**2 + \
                        (self.controller.state_ref[1] - self.controller.state_asv[1])**2)
        DIST_THRESHOLD = rospy.get_param('/dist_threshold', 1.0)
        if dist < DIST_THRESHOLD:
            return Finished()
        elif event == 'start_over':
            return Start()
        else:
            return self

    def get_distance(self, ang, nbr_of_points=5):
        '''Getting the mean distance to the specified angle
            ang: angle to look for shortest range
            nbr_of_points: number of readings to average over'''
        state_asv = self.controller.state_asv
        ranges = self.ranges
        lidar_inc = 2*math.pi / len(ranges)
        print('inc', lidar_inc)
        print('ranges[350]', ranges[350])
        nbr = int(math.floor(nbr_of_points/2))
        print('angle to wall ', ang)
        print('theta ', state_asv[2])
        index = int((self.controller.angleDiff(ang - state_asv[2]) \
                                                    + math.pi)/lidar_inc) % len(ranges)
        print('angleDiff ', self.controller.angleDiff(ang - state_asv[2]))
        print('everything excpt mod', int((self.controller.angleDiff(ang - state_asv[2]) + math.pi)/lidar_inc))
        print('index ', index)
        range_sum = ranges[index]
        for i in range(1,nbr+1):
            range_sum += math.cos(lidar_inc*i)*(ranges[(index+i) % len(ranges)] \
                            + ranges[(index-i) % len(ranges)])

        # return mean of nbr_of_points (uneven) closest points
        return range_sum/(2*nbr+1)

    def calc_control(self):
        '''remember to update the controller before calling this'''
        #print('controller wp' , self.controller.wayPoints)
        self.controller.state_ref[0] = self.xref
        self.controller.state_ref[1] = self.yref
        return self.controller.calc_control()

class Finished(State):
    '''Keep position indefinately'''
    def __init__(self):
        State.__init__(self)
        self.controller = PI_controller.PI_controller()

    def on_event(self, event):
        return self

    def calc_control(self):
        '''remember to update the controller before calling this'''
        self.controller.destinationReached(True)
        # self.controller.state_ref = self.controller.state_asv
        return self.controller.calc_control()