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
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Point, PointStamped
import random


########## Smart LiDAR controller state machine #######
class Smart_LiDAR_Controller(Generic_Controller):
    """
    Program to either go upstream or doing transects using the LiDAR readings.
    Uses either the waypoint/PI controller or the transect controller depending
    on what state it is in. Inherits the Generic_Controller

    rosparam set /smart/mode Upstream - Go upstream at desired place in river
    rosparam set /smart/mode Transect - Do transects
    """

    def __init__(self):
        Generic_Controller.__init__(self)
        self.state = Start()
        self.ranges = []        # ranges from lidar
        self.lidar_inc = 1.0    # angle increment between measurements in ranges, initialized as 1 to avoid division by zero
        self.goback_state = Start() # keeps track of which state to go back to after being in Idle state
        self.goback_bool = False    # helps keeping track of which state to go back to after being in Idle state
        self.state_pub = rospy.Publisher('smart/state', String, queue_size=10)
        self.point_pub = rospy.Publisher('/ref/point', PointStamped, queue_size=10)

    def update_lidar(self, ranges, inc):
        """
        Update lidar readings.

        Args:
            ranges (float[]): vector with ranges
            inc (float): angle increment
        """

        self.ranges = ranges
        self.lidar_inc = 2*math.pi/len(ranges)
        # self.lidar_inc = inc      we have had problems using this, somewhere this inc is given the wrong value

    def calc_control(self):
        """
        Calculates the control output. Uses either waypoint/PI or the transect
        controller to calculate it.

        Returns:
            u_thrust (float): control output for the thrusters.
            u_rudder (float): control output for the servo.
        """
        # Update the controller
        # Important some controllers might use their own info, such as Transect
        self.state_pub.publish(self.state.__str__())
        if (rospy.get_param('smart/idle', False) or self.destReached) and self.state.__str__() != "Idle":
            ''' Sets the boat in the Idle state which keeps position,
                remembers the last state to be able to continue the run '''
            self.goback_state = self.state
            self.state = Idle(self.state.controller.state_asv)
            self.goback_bool = True
        elif (not (rospy.get_param('smart/idle', False) or self.destReached)) and self.goback_bool:
            ''' Continue run after being in the Idle state '''
            self.state = self.goback_state
            self.goback_bool = False
    	if rospy.get_param('restart', False):
            ''' Restart the run '''
    	    self.state = Start()
    	    rospy.set_param('restart', False)

        self.state.update_controller_var(self.state_asv, self.state_ref,
                self.v_asv, self.target_index, self.wayPoints, self.current)
        self.state.controller.destinationReached(self.destReached)
        # state transition
        self.state = self.state.on_event('Run')
        self.state.update_lidar(self.ranges, self.lidar_inc)

        # Visualize reference point in Rviz
        refpoint = PointStamped()
        refpoint.header.stamp = rospy.get_rostime()
        refpoint.header.frame_id = "/map"
        refpoint.point.x = self.state.controller.state_ref[0]
        refpoint.point.y = self.state.controller.state_ref[1]
        self.point_pub.publish(refpoint)

        print('STATE: ', self.state)
        return self.state.calc_control()

########## Specific states ############

class Start(State):
    """
    Initial state of the controller. Drives to a specified start point.
    Inherits State
    """

    def __init__(self):
        State.__init__(self)
        self.controller = PI_controller.PI_controller()
        self.xref = rospy.get_param('/start/x', 0.0)
        self.yref = rospy.get_param('/start/y', 0.0)

    def on_event(self, event):
        """
        Determins the next state in the control program.

        Args:
            String: a string that describes an event happend in the asv
        Returns:
            State: the next state
        """
        DIST_THRESHOLD = rospy.get_param('/dist_threshold', 1.0)
        dist = math.sqrt((self.controller.state_ref[0] - self.controller.state_asv[0])**2 \
                    + (self.controller.state_ref[1] - self.controller.state_asv[1])**2)

        if dist < DIST_THRESHOLD:
            mode = rospy.get_param('/smart/mode', "default")
            if mode == "Upstream":
                return Explore(self.ranges)
            elif mode == "Transect":
                return Hold(self.controller.state_asv)
            else:
                # how to print in a good way John??
                rospy.logdebug("Not a valid mode")
                return Home(self.ranges, self.controller.state_asv, self.controller.current)

        else:
            return self

    def calc_control(self):
        """
        Calculates the control output. Uses the waypoint/PI
        controller to calculate it.
        Remember to update the controller before calling this

        Returns:
            u_thrust (float): control output for the thrusters.
            u_rudder (float): control output for the servo.
        """
        # Right now the start point is chosen by updating controller.state_ref
        # This can be done for instance from Rviz
        # If this should be done via rosparams, uncomment the following lines

        # self.xref = rospy.get_param('/start/x', 0.0)
        # self.yref = rospy.get_param('/start/y', 0.0)
        # self.controller.state_ref[0] = self.xref
        # self.controller.state_ref[1] = self.yref
        return self.controller.calc_control()


class Hold(State):
    """
    Second state of the controller when in transect mode.
    Holds position for a specified time to figure out the current.
    Inherits State
    """

    def __init__(self, state_asv_prev):
        State.__init__(self)
        self.controller = PI_controller.PI_controller()
        self.controller.state_asv = state_asv_prev
        self.controller.state_ref = self.controller.state_asv
        self.start_time = rospy.get_rostime()
        self.waitTime = rospy.get_param('/hold/wait_time', 5.0)

        # for averaging:
        # set average and reset (?) param true

    def on_event(self, event):
        """
        Determins the next state in the control program.

        Args:
            String: a string that describes an event happend in the asv
        Returns:
            State: the next state
        """
        self.waitTime = rospy.get_param('/hold/wait_time', 5.0)
        if (rospy.get_rostime() - self.start_time).to_sec() > self.waitTime:
            #  The boat has held position for the specified time
            # for averaging:
            # set average param false (?)
            return Transect(last_controller=self.controller, ranges=self.ranges,
                                lidar_inc=self.lidar_inc)
        else:
            return self

    def calc_control(self):
        """
        Calculates the control output. Uses the waypoint/PI
        controller to calculate it.
        Remember to update the controller before calling this

        Returns:
            u_thrust (float): control output for the thrusters.
            u_rudder (float): control output for the servo.
        """
        self.controller.destinationReached(True)
        return self.controller.calc_control()

class Idle(State):
    """
    Holds position, activated by user. Always returns itself, exited from Smart_LiDAR_Controller
    Inherits State
    """
    def __init__(self, state_asv_prev):
        State.__init__(self)
        self.controller = PI_controller.PI_controller()
        self.controller.state_asv = state_asv_prev
        self.controller.state_ref = self.controller.state_asv

    def on_event(self, event):
        """
        Determins the next state in the control program.

        Args:
            String: a string that describes an event happend in the asv
        Returns:
            State: the next state
        """
        return self

    def calc_control(self):
        """
        Calculates the control output. Uses the waypoint/PI
        controller to calculate it.
        Remember to update the controller before calling this

        Returns:
            u_thrust (float): control output for the thrusters.
            u_rudder (float): control output for the servo.
        """
        self.controller.destinationReached(True)
        return self.controller.calc_control()

class Transect(State):
    """
    Third state of the controller when in transect mode.
    Perform transect on the spot using the transect controller.
    Inherits State
    """

    def __init__(self, last_controller=None, ranges=None, lidar_inc=None, transect = True, dir=True, trans_cnt=None):
        State.__init__(self)
        self.controller = transect_controller.Transect_controller(controller=last_controller)
        # self.controller = PI_controller.PI_controller(controller=last_controller)
        if trans_cnt == None:
             # If this is the first transect
            self.transect_cnt = 0                   # keep track of nbr of transects made
            self.start_time = rospy.get_rostime()   # keep track of time during which transects has been made
        else:
            # If transects has been performed before,
            # i.e if the boat has been going upstream between transects
            self.transect_cnt = trans_cnt[0]        # keep track of nbr of transects made
            self.start_time = trans_cnt[1]          # keep track of time during which transects has been made

        self.run_time = rospy.get_param('/transect/run_time', 300.0)        # max run time
        self.max_transect = rospy.get_param('/transect/max_transect', 2)    # max nbr of transects
        self.dist_th = rospy.get_param('/transect/dist_threshold', 15.0)    # distance to shore
        self.trans_per_upstr = self.transect_cnt + rospy.get_param('/upstream/cnt_per', 1) # number of full transects, 0 gives the first "half transect"
        self.upstream = rospy.get_param('/upstream', True)      # if True the boat will go upstream inbetween transects

        self.direction = dir #False - look at shore to the left, True-look at shore to the right
        self.p1 = Point()   # transect points
        self.p2 = Point()
        self.last_turn = rospy.get_rostime()        # wait for short while after turning before looking at lidar
        self.ranges = ranges
        self.lidar_inc = lidar_inc

        current = self.controller.current # fix the averaging
        current_angle = current[1]

        if transect:
            # calculate transect once upon initialization, reason it is in if statement is that this state also is used to get distances
            [self.p1, self.p2] = self.calculate_transect(current_angle)
            # IMPORTANT: Transect state disregard target_index, wayPoints information
            # from the top controller. It instead has its own
            self.controller.transect = True
            self.controller.theta_p = self.transect_angle       # angle of the transect line, used for distance measurements

            if self.direction:
                self.target_index = 0
            else:
                self.target_index = 1
            self.wayPoints = [self.p1, self.p2]

    def on_event(self, event):
        """
        Determins the next state in the control program.

        Args:
            String: a string that describes an event happend in the asv
        Returns:
            State: the next state
        """
        if self.direction:
            ''' to determine what angle for the lidar measurements'''
            angle = self.transect_angle
        else:
            angle = self.controller.angleDiff(self.transect_angle + math.pi)

        dist = self.get_distance(angle, 41)     # distance to shore at angle averaged over 41 points

        if (rospy.get_rostime() - self.start_time).to_sec() > self.run_time:
            # Max time reached, return home
            return Home(self.ranges, self.controller.state_asv, self.controller.current)
        elif dist < rospy.get_param('/transect/dist_threshold', 15.0):
            # Distance to shore to small, either go back home, change direction or go upstream
            self.direction = not self.direction
            self.last_turn = rospy.get_rostime()
            self.transect_cnt += 1
            if self.transect_cnt >= self.max_transect:
                # Max nbr of transects reached, return home
                return Home(self.ranges, self.controller.state_asv, self.controller.current)
            if (self.direction):
                self.target_index = 0
            else:
                self.target_index = 1
            if self.upstream and self.transect_cnt > self.trans_per_upstr:
                # Going upstream inbetween transects and max transects at this place reached
                trans_cnt = [self.transect_cnt, self.start_time]
                return Upstream(self.ranges, self.controller.state_asv, self.controller.current, self.direction, trans_cnt)
            else:
                return self
        else:
            return self

    def calc_control(self):
        """
        Calculates the control output. Uses the transect
        controller to calculate it.
        Remember to update the controller before calling this

        Returns:
            u_thrust (float): control output for the thrusters.
            u_rudder (float): control output for the servo.
        """
        self.controller.wayPoints = self.wayPoints
        self.controller.target_index = self.target_index
        ref = self.calc_refPoint()
        self.controller.state_ref[0] = ref[0]
        self.controller.state_ref[1] = ref[1]
        return self.controller.calc_control()

    def get_distance(self, ang, nbr_of_points=5):
        """
        Getting the mean distance from the LiDAR at the specified angle

        Args:
            ang (float): angle to look for shortest range (in global frame)
            nbr_of_points (int): number of readings to average over, default=5
        Returns:
            float: average distance of specified number of points at specified angle
        """
        state_asv = self.controller.state_asv
        lidar_inc = 2*math.pi / len(self.ranges)
        lidar_off = rospy.get_param('lidar_offset', 0.0) # ofset between lidar and boat in radians
        nbr = int(math.floor(nbr_of_points/2))          # get nbr of points above/below wanted angle for averaging
        index = int((self.controller.angleDiff(ang - state_asv[2]) \
                + math.pi)/lidar_inc) % len(self.ranges)    # index of angle in ranges
        range_sum = self.ranges[index]      # sum of ranges, for averaging
        for i in range(1,nbr+1):
            range_sum += math.cos(lidar_inc*i)*(self.ranges[(index+i) % len(self.ranges)] \
                + self.ranges[(index-i) % len(self.ranges)])    # add point above/below angle to sum, for averaging

        # return mean of nbr_of_points (uneven) closest points
        return range_sum/(2*nbr+1)

    def calc_refPoint(self):
        """
        Returns the reference point for the transect controller

        Returns:
            x (float): x coordinate
            y (float): y coordinate
        """

        '''calculates the reference points if PI controller is used for transects.
            projects boats position onto transect line. ref point is point delta m
            ahead of this point on the line. This code does not work at the moment, for some reason the direction
                is not changed in reality. But works in simulation...'''

        # pos = np.array([self.controller.state_asv[0], self.controller.state_asv[1]])
        # line = np.array([math.cos(self.transect_angle), math.sin(self.transect_angle)])
        # proj = np.dot(pos, line) * line #/ np.dot(line, line) * line if line is not normalized, use this if line is not [cos(),sin()]
        # delta = proj - pos
        # if self.direction:
        #     line = np.array([math.cos(self.transect_angle + math.pi), math.sin(self.transect_angle + math.pi)])
        #     pos = np.array([self.controller.state_asv[0], self.controller.state_asv[1]])
        #     proj = np.dot(pos, line) * line #/ np.dot(line, line) * line if line is not normalized, use this if line is not [cos(),sin()]
        #     ref = proj + self.transect_center - rospy.get_param('/transect/delta_ref', 2.5) * line #make sure line is normalized
        # else:
        #     line = np.array([math.cos(self.transect_angle), math.sin(self.transect_angle)])
        #     pos = np.array([self.controller.state_asv[0], self.controller.state_asv[1]])
        #     proj = np.dot(pos, line) * line #/ np.dot(line, line) * line if line is not normalized, use this if line is not [cos(),sin()]
        #     ref = proj + self.transect_center = rospy.get_param('/transect/delta_ref', 2.5) * line #make sure line is normalized
        # return ref

        #This is reference when transect controller is used
        if self.direction:
            ref = [self.p1.x, self.p1.y]
        else:
            ref = [self.p2.x, self.p2.y]
        return ref


    #Calculates two transect points (on land) creating a line perpendicular to current
    def calculate_transect(self, theta_c):
        """
        Calculates two transect points (on land), creating a line perpendicular to the current

        Args:
            theta_c (float): current angle in global frame
        Returns:
            point1 (Point): The first transect point
            point2 (Point): The second transect point
        """
        state_asv = self.controller.state_asv
        point1 = Point()
        point2 = Point()
        # self.transect_center = [self.controller.state_asv[0], self.controller.state_asv[1]]       used in ref calculations when transect with PI

        # sample cerain number of points from the sides of the current angle
        distL = self.get_distance(theta_c + math.pi/2, 21)
        distR = self.get_distance(theta_c - math.pi/2, 21)

        # generate points using simple trig, 10m from shore line
        point1.x = state_asv[0] + (distR + 10) * math.cos(theta_c - math.pi/2)
        point1.y = state_asv[1] + (distR + 10) * math.sin(theta_c - math.pi/2)
        point1.z = 0.0
        point2.x = state_asv[0] + (distL + 10) * math.cos(theta_c + math.pi/2)
        point2.y = state_asv[1] + (distL + 10) * math.sin(theta_c + math.pi/2)
        point2.z = 0.0

        self.transect_angle = theta_c - math.pi/2       # angle of the line, used for lidar measurements

        return [point1, point2]

    def too_close(self, dir):
        """
        Looks at the distances to right or left. If a number of points are closer than a specified threshold returns true.

        Args:
            dir (bool): direction to look at
        Returns:
            bool: True if shore is deemed to be closer than threshold
        """
        ''' This function is not used at the moment. get_distance worked better so is used instead '''
        state_asv = self.controller.state_asv
        ranges = self.ranges
        inc = self.lidar_inc
        wayPoints = self.wayPoints # self.controller.wayPoints is overwritten every loop!

        self.dist_th = rospy.get_param('/transect/dist_threshold', 25.0)
        dist_th = self.dist_th

        theta_p = self.controller.angleDiff(math.atan2(wayPoints[0].y \
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

        #If more than a specified number of the points in the region specified above are within threshold, turn around
        if (int((start_ang + math.pi)/inc) <  int((end_ang + math.pi)/inc)):
            dists = ranges[np.arange(int((start_ang + math.pi)/inc) % len(ranges), int((end_ang + math.pi)/inc) % len(ranges))]
        else:
            dists1 = ranges[np.arange(int((start_ang + math.pi)/inc), len(ranges))]
            dists2 = ranges[np.arange(0, int((end_ang + math.pi)/inc))]
            dists = np.concatenate((dists1, dists2))
        close = np.nonzero(dists < dist_th)

        if (rospy.get_rostime() - self.last_turn).to_sec() < 2.0:
            return False
        else:
            return len(close[0]) > 10 #is this a reasonable way of doing it? now turns if more than 10 values are to close...

    def get_transect(self):
        return self.wayPoints


class Upstream(State):
    """
    Fourth state of the controller when in transect mode if upstream is activated.
    Moves upstream in the middle, or at a specified fraction, of the river to a point
    a specified distance upstream.
    Inherits State
    """
    def __init__(self, lidar_readings, last_state, current, dir, trans_cnt):
        State.__init__(self)
        self.ranges = lidar_readings
        self.controller = PI_controller.PI_controller()
        self.controller.state_asv = last_state
        self.controller.current = current
        self.direction = dir
        self.trans_cnt = trans_cnt
        self.dist_calc = Transect(last_controller=self.controller, ranges=self.ranges,
            lidar_inc=self.lidar_inc, transect=False)       # uses functions in the Transect state to get distances
        self.update_ref()

    def update_ref(self):
        """
        Updates the reference point to be in the middle (or fraction specified
        by rosparam) of the river a specified distance upstream
        """
        self.dist_calc.ranges = self.ranges
        d_left = self.dist_calc.get_distance(self.controller.current[1] + math.pi/2, 21)    # distance to the left
        d_right = self.dist_calc.get_distance(self.controller.current[1] - math.pi/2, 21)   # distance to the right
        dist_mid = (d_right - d_left)/rospy.get_param('/waypoint/fraction', 3)      # distance to specified fraction from the boat

        theta_p = self.controller.current[1] - np.sign(dist_mid) * math.pi/2    # angle perpendicular to current
        dist_upstream = np.sign(math.sin(self.controller.current[1])) * rospy.get_param('/dist_upstream', 5.0)  # distance upstream, sign to make sure it works in any direction
        theta_dest = self.controller.angleDiff(theta_p + np.sign(dist_upstream*dist_mid)*(math.atan2(abs(dist_upstream), abs(dist_mid))))   # angle to point in global frame
        dist = math.sqrt(dist_mid**2 + dist_upstream**2)        # distance to ref point
        self.xref = self.controller.state_asv[0] + dist * math.cos(theta_dest)
        self.yref = self.controller.state_asv[1] + dist * math.sin(theta_dest)

    def on_event(self, event):
        '''
        Determins the next state in the control program.

        Args:
            String: a string that describes an event happend in the asv
        Returns:
            State: the next state
        '''
        dist = math.sqrt((self.controller.state_ref[0] - self.controller.state_asv[0])**2 +
            (self.controller.state_ref[1] - self.controller.state_asv[1])**2)
        DIST_THRESHOLD = rospy.get_param('/dist_threshold', 1.0)
        run_time = rospy.get_param('/transect/run_time', 100.0)
        if (rospy.get_rostime() - self.trans_cnt[1]).to_sec() > run_time:
            # if max time reached, return home
            return Home(self.ranges, self.controller.state_asv, self.controller.current)
        if dist < DIST_THRESHOLD:
            # if close to ref point, do transects again
            return Transect(last_controller=self.controller, ranges=self.ranges,
                                lidar_inc=self.lidar_inc, dir=self.direction, trans_cnt=self.trans_cnt)
        else:
            return self

    def calc_control(self):
        """
        Calculates the control output. Uses the waypoint/PI
        controller to calculate it.
        Remember to update the controller before calling this

        Returns:
            u_thrust (float): control output for the thrusters.
            u_rudder (float): control output for the servo.
        """
        self.controller.state_ref[0] = self.xref
        self.controller.state_ref[1] = self.yref
        return self.controller.calc_control()

class Home(State):
    """
    Last state of the controller. Returns the boat to the home postion (downstream).
    Starts by moving downstream in the middle of the river. When the angle to the home positon
    is within some interval the boat goes towards the home position.
    Inherits State

    "Hem sweet hem (home ljuva home)"

    """

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
        """
        Updates the reference point to be in the middle of the river a
        specified distance downstream. If go_home=True the reference point is set
        to be the home position.

        Args:
            go_home (bool): When True the home position is the reference point, default=False
        """
        if go_home:
            # home position is the reference point
            self.xref = self.home[0]
            self.yref = self.home[1]
        else:
            # calculate reference point in a similar way as the upstream update_ref
            self.dist_calc.ranges = self.ranges
            self.dist_calc.controller.state_asv = self.controller.state_asv
            d_left = self.dist_calc.get_distance(self.controller.current[1] + math.pi/2, 21)
            d_right = self.dist_calc.get_distance(self.controller.current[1] - math.pi/2, 21)
            dist_mid = (d_right - d_left)/rospy.get_param('/waypoint/fraction', 2)
            theta_p = self.controller.current[1] - np.sign(dist_mid) * math.pi/2
            dist_downstream = np.sign(math.sin(self.controller.current[1])) * rospy.get_param('/dist_downstream', 2.0)
            theta_dest = self.controller.angleDiff(theta_p - np.sign(dist_downstream*dist_mid)* math.atan2(abs(dist_downstream), abs(dist_mid)))
            dist = math.sqrt(dist_mid**2 + dist_downstream**2)
            self.xref = self.controller.state_asv[0] + dist * math.cos(theta_dest)
            self.yref = self.controller.state_asv[1] + dist * math.sin(theta_dest)

    def on_event(self, event):
        '''
        Determins the next state in the control program.

        Args:
            String: a string that describes an event happend in the asv
        Returns:
            State: the next state
        '''
        theta_home = self.controller.state_asv[2] - math.atan2(self.home[1] - self.controller.state_asv[1], \
                self.home[0] - self.controller.state_asv[0])    # angle to home point
        if abs(theta_home) <= 2*math.pi/3:
            # if angle to home is smaller than some specified angle, boat is assumed to be going downstream.
            go_home = True
        else:
            go_home = False
        self.update_ref(go_home)
        dist = math.sqrt((self.controller.state_ref[0] - self.controller.state_asv[0])**2 + \
                        (self.controller.state_ref[1] - self.controller.state_asv[1])**2)
        DIST_THRESHOLD = rospy.get_param('/dist_threshold', 1.0)
        if dist < DIST_THRESHOLD:

            return Finished(home)
        elif event == 'start_over':
            return Start()
        else:
            return self

    def calc_control(self):
        """
        Calculates the control output. Uses the waypoint/PI
        controller to calculate it.
        Remember to update the controller before calling this

        Returns:
            u_thrust (float): control output for the thrusters.
            u_rudder (float): control output for the servo.
        """
        self.controller.state_ref[0] = self.xref
        self.controller.state_ref[1] = self.yref
        return self.controller.calc_control()

class Finished(State):
    """Keep position indefinately"""
    def __init__(self, home_pos):
        State.__init__(self)
        self.controller = PI_controller.PI_controller()
        self.controller.state_ref[0] = home_pos[0]
        self.controller.state_ref[1] = home_pos[1]


    def on_event(self, event):
        '''
        Determins the next state in the control program.

        Args:
            String: a string that describes an event happend in the asv
        Returns:
            State: the next state
        '''
        return self

    def calc_control(self):
        """
        Calculates the control output. Uses the waypoint/PI
        controller to calculate it.
        Remember to update the controller before calling this

        Returns:
            u_thrust (float): control output for the thrusters.
            u_rudder (float): control output for the servo.
        """
        self.controller.destinationReached(True)
        return self.controller.calc_control()

class Explore(State):
    """
    Third state of the controller when in upstream mode.
    Moves upstream in the middle, or at a specified fraction, of the river
    for a specified time or distance.
    Inherits State
    """
    def __init__(self, lidar):
        State.__init__(self)
        self.ranges = lidar

        self.start_time = rospy.get_rostime()
        self.controller = PI_controller.PI_controller()
        self.dist_calc = self.dist_calc = Transect(last_controller=self.controller, ranges=self.ranges,
                lidar_inc=self.lidar_inc, transect=False)   # for distance calculations
        self.dist_travelled = 0.0
        self.calc_pub = rospy.Publisher('/smart/calcs', Float32MultiArray, queue_size=10)

        if len(self.ranges) == 0:
            self.ranges = [0]       # prevent division by zero

        self.update_ref()

    def on_event(self, event):
        '''
        Determins the next state in the control program.

        Args:
            String: a string that describes an event happend in the asv
        Returns:
            State: the next state
        '''
        self.update_ref()
        self.dist_travelled += self.controller.vel_robotX * 0.2
        if (rospy.get_rostime() - self.start_time).to_sec() > rospy.get_param('/explore/max_time', 120.0) or rospy.get_param('go_home', False): #self.dist_travelled > rospy.get_param('/max_distance', 30.0):
            # If max time reached or go_home rosparam set true
            #problems with distance travelled, maybe use GPS instead of "integrating" like we've done
            return Home(self.ranges, self.controller.state_asv, self.controller.current)
        else:
            return self

    def update_ref(self):
        """
        Updates the reference point to be in the middle (or fraction specified
        by rosparam) of the river.
        """
        cals_msg = Float32MultiArray()
        self.dist_calc.ranges = self.ranges
        self.dist_calc.controller.state_asv = self.controller.state_asv
        d_left = self.dist_calc.get_distance(self.controller.current[1] + math.pi/2, 21)
        d_right = self.dist_calc.get_distance(self.controller.current[1] - math.pi/2, 21)
        dist_mid = (d_right - d_left)/rospy.get_param('/waypoint/fraction', 2)
        theta_p = self.controller.current[1] - np.sign(dist_mid) * math.pi/2
        dist_upstream = np.sign(math.sin(self.controller.current[1])) * rospy.get_param('/dist_upstream', 2.0)
        theta_dest = self.controller.angleDiff(theta_p + np.sign(dist_upstream*dist_mid)*(math.atan2(abs(dist_upstream), abs(dist_mid))))
        dist = math.sqrt(dist_mid**2 + dist_upstream**2)

        self.xref = self.controller.state_asv[0] + dist * math.cos(theta_dest)
        self.yref = self.controller.state_asv[1] + dist * math.sin(theta_dest)

        cals_msg.data = [d_left, d_right, dist_mid, theta_p, theta_dest, self.xref, self.yref]
        self.calc_pub.publish(cals_msg)


    def d2t(self):
        """
        Calculates the distance to the current waypoint.

        Returns:
            float: distance to waypoint
        """
        return math.sqrt((self.controller.state_asv[0] - self.xref)**2 + (self.controller.state_asv[1] - self.yref)**2)

    def calc_control(self):
        """
        Calculates the control output. Uses the waypoint/PI
        controller to calculate it.
        Remember to update the controller before calling this

        Returns:
            u_thrust (float): control output for the thrusters.
            u_rudder (float): control output for the servo.
        """
        self.controller.state_ref[0] = self.xref
        self.controller.state_ref[1] = self.yref
        return self.controller.calc_control()
