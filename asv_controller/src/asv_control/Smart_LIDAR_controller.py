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
        self.lidar_inc = inc

    def calc_control(self):
        # Update the controller
        self.state.update_controller_var(self.state_asv, self.state_ref, \
                self.v_asv, self.target_index, self.wayPoints, self.current)
        self.state.controller.destinationReached(self.destReached)
        
        # state transition       
        self.state = self.state.on_event('Run')
        self.state.update_lidar(self.ranges, self.lidar_inc)
        
        return self.state.calc_control()

########## Specific states ############

class Start(State):
    ''' This initial state of the controller drives the robot to a designated
        point ''' 
    def __init__(self):
        State.__init__(self)
        self.controller = PI_controller.PI_controller()

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
        return self.controller.calc_control()

    
class Hold(State):
    '''Hold at the point until the asv figures out the current'''
    def __init__(self):
        State.__init__(self)
        self.controller = PI_controller.PI_controller()
        self.start_time = rospy.get_rostime()
        self.waitTime = rospy.get_param('/wait_time', 5.0)

    def on_event(self, event):
        if (rospy.get_rostime() - self.start_time).to_sec() > self.waitTime:
            return Transect(last_controller=self.controller, ranges=self.ranges,
                                lidar_inc=self.lidar_inc)
        else:
            return self

    def calc_control(self):
        '''remember to update the controller before calling this'''
        self.controller.destinationReached(True)
        self.controller.state_ref = self.controller.state_asv
        return self.controller.calc_control()

class Transect(State):
    '''Perform transect on the spot'''

    def __init__(self, last_controller=None, ranges=None, lidar_inc=None):
        State.__init__(self)
        self.controller = transect_controller.Transect_controller(controller=last_controller)
        
        self.start_time = rospy.get_rostime()
        self.run_time = 120.0
        self.transect_cnt = 0
        self.max_transect = 5
        self.transect_time_ref = 5.0
        self.direction = False #true - look at shore to the left, false - look at shore to the right
        self.p1 = GPS_data()
        self.p2 = GPS_data()
        
        self.ranges = ranges
        self.lidar_inc = lidar_inc

        # calculate transect once upon initialization
        current = self.controller.current # fix the averaging
        current_angle = current[1]
        [self.p1, self.p2] = self.calculate_transect(current_angle)
        self.controller.wayPoints = [self.p1, self.p2]

        # IMPORTANT: Transect state disregard target_index, wayPoints information 
        # from the top controller. It instead has its own
        self.target_index = 1 
        self.wayPoints = [self.p1, self.p2]

    def on_event(self, event):
        if event == 'transect_time_elapsed' or event == 'num_transected_reached':
            # deal with this later
            return Upstream()
        elif (rospy.get_rostime() - self.start_time).to_sec() > self.run_time \
                                      or self.transect_cnt > self.max_transect:
            return Home()
        elif self.too_close(self.direction):
            self.direction = not self.direction
            self.transect_cnt += 1
            if (self.direction):
                self.target_index = 0
            else:
                self.target_index = 1            
            return self
        else:
            return self


    def calc_control(self):
        '''remember to update the controller before calling this'''
        
        # transect calculate its own waypoints and targetIndex here
        self.controller.wayPoints =self.wayPoints
        self.controller.target_index = self.target_index

        return self.controller.calc_control()

    def get_distance(self, ang, nbr_of_points=5):
        '''Getting the mean distance to the specified angle
            ang: angle to look for shortest range
            nbr_of_points: number of readings to average over'''
        state_asv = self.controller.state_asv
        ranges = self.ranges
        lidar_inc = self.lidar_inc

        nbr = int(math.floor(nbr_of_points/2))
        inc = lidar_inc

        index = int((self.controller.angleDiff(ang - state_asv[2] ) \
                                                    + math.pi)/inc)
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
        #print('distL', distL)
        distR = self.get_distance(theta_c - math.pi/2, 21)
        #print('distR', distR)

        # generate points using simple trig
        point1.x = state_asv[0] + (distR + 10) * math.cos(theta_c - math.pi/2)
        point1.y = state_asv[1] + (distR + 10) * math.sin(theta_c - math.pi/2)
        point2.x = state_asv[0] + (distL + 10) * math.cos(theta_c + math.pi/2)
        point2.y = state_asv[1] + (distL + 10) * math.sin(theta_c + math.pi/2)

        #print(point1)
        #print(point2)

        #how should the points be saved for transect controller??
        return [point1, point2]

    def too_close(self, dir):
        ''' Compares distance to shore on either right or left side to the threshold.
            If the number of points exceeds some number, turn around '''
        state_asv = self.controller.state_asv
        ranges = self.ranges        
        inc = self.lidar_inc
        wayPoints = self.controller.wayPoints

        dist_th = rospy.get_param('dist_th', 3.0)

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


class Upstream(State):
    '''Move upstream against the current'''
    def __init__(self):
        State.__init__(self)
        self.controller = PI_controller.PI_controller()

    def on_event(self, event):
        if event == 'next_point_reached':
            return Transect()
        elif event == 'go_home':
            return Home()
        else:
            return self

class Home(State):
    '''Go back to hem (home)! Hem sweet hem.'''
    def __init__(self):
        State.__init__(self)
        self.controller = PI_controller.PI_controller()

    def on_event(self, event):
        if event == 'start_over':
            return Start()
        else:
            return self






