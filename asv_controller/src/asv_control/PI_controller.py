#!/usr/bin/env python

import rospy
import math
import numpy as np
import time
from Generic_Controller import Generic_Controller
from asv_controller.msg import PI_states

# inherited from the Generic Controller class
# members:
    # state_asv, state_ref, current, v_asv, wayPoints, target_index,
    # destReached
# Functions:
    # destinationReached
    # update_variable
    # angleDiff

class PI_controller(Generic_Controller):
    """
    Waypoint tracking controller. PI controller for rudder angle and thrust.
    Looks at the current angle to keep boat pointing upstream.
    """

    def __init__(self, controller=None, trans_angle=None):
        Generic_Controller.__init__(self, controller=controller, trans_angle=trans_angle) # initialize from generic controller

        # parameters specific to PI_controller
        self.I_thrust = 0.0
        self.I_rudder = 0.0
        self.h = 0.2
        self.V_REF = rospy.get_param('v_ref', 0.5)
        self.VEL_THRESHOLD = rospy.get_param('/v_threshold', 0.2)
        self.K_t = rospy.get_param('thrust/K', 500.0)
        self.Ti_t = rospy.get_param('thrust/Ti', 1.0)
        self.K_r = rospy.get_param('rudder/K', 150.0)
        self.Ti_r = rospy.get_param('rudder/Ti', 10.0)
        self.v_robotX = 0.0                         #For integration to get distance upstream, maybe better to use GPS instead

        self.debug_messages = PI_states()
        self.debug_pub = rospy.Publisher('pi_controller/states', PI_states, queue_size=10)



    def d2t(self):
        """
        Calculates the distance to the current waypoint.

        Returns:
            float: distance to waypoint
        """
        return math.sqrt((self.state_asv[0] - self.state_ref[0])**2 +
                        (self.state_asv[1] - self.state_ref[1])**2)

    # TODO: maybe make this an abstract method for generic controller
    def calc_control(self):
        """
        Calculates the control outputs.

        Returns:
            u_thrust (float): control output for the thrusters.
            u_rudder (float): control output for the servo.
        """
        rospy.logdebug("Desired wp: " + str(self.state_ref))
        rospy.logdebug("vel(x,y): " + str(self.v_asv))

        # update the param for live change from the user
        self.V_REF = rospy.get_param('v_ref', 0.5)
        self.VEL_THRESHOLD = rospy.get_param('/v_threshold', 0.2)
        v_ref = self.V_REF
        self.dist_threshold = rospy.get_param('/dist_threshold', 1.0)

        # If true v_ref depends linearly on the distance to the waypoint, this is used when close to the point
        lin_v = False

        '''transform velocities to robots coordinate system'''
        rot = np.array([[np.cos(self.state_asv[2]), np.sin(self.state_asv[2])],
            [-np.sin(self.state_asv[2]), np.cos(self.state_asv[2])]])           #rotation matrix
        vel_unrot = np.array([[self.v_asv[0]],[self.v_asv[1]]])                 #velocity in global frame
        vel_robot = np.matmul(rot, vel_unrot)                                   #velocity in local frame
        self.vel_robotX = vel_robot[0]

        #fix so I parts are reset when /motor/sim is changed to False
        if rospy.get_param('/I/reset', False):
            rospy.set_param('/I/reset', False) # change it back to true (stop resetting)
            self.I_rudder = 0.0
            self.I_thrust = 0.0

        '''evaluate rotation of robot compared to current'''
        e_heading = self.angleDiff(self.current[1] - self.state_asv[2])     #difference between current and heading of the boat
        rospy.logdebug(e_heading)
        ang_dir = self.state_asv[2]

        v = math.sqrt(self.v_asv[0]**2 + self.v_asv[1]**2)

        if abs(e_heading) > math.pi/3 and self.current[0] > 0.2:
            '''turn robot back towards current if it points to much downward'''
            rospy.logdebug('aligning to current')
            e_ang = e_heading
            self.debug_messages.action = "aligning to current"

        elif self.d2t() < self.dist_threshold/3:
            '''holding position'''
            e_ang = e_heading
            v_ref = 0.0
            self.debug_messages.action = 'holding'

        else:
            rospy.logdebug('calculates PI outputs')
            self.debug_messages.action = "going upstream"

            des_angle = math.atan2(self.state_ref[1] - self.state_asv[1],
                    self.state_ref[0] - self.state_asv[0])          #angle to waypoint

            '''decide which feedback to use for angle'''
            if v < self.VEL_THRESHOLD or vel_robot[0,0] < 0.0:
                '''if moving slowly or backwards, use heading as feedback'''
                ang_dir = self.state_asv[2]
                # for future:   could be possible to use GPS as feedback when moving backwards as well
                #               Would need testing, but could give better results

            else:
                '''else use heading from GPS'''
                ang_dir = self.v_asv[2]

            if self.d2t() < rospy.get_param('/d2t', 2*self.dist_threshold):
                '''if distance to waypoints is smaller v_ref is scaled linearly
                    with the distance to the waypoint, the maximum difference between
                    current and heading is scaled with the same factor.
                    To reduce oscillations around this point'''
                lin_factor = (self.d2t() - self.dist_threshold)/rospy.get_param('/d2t', 1.5)
                v_ref = lin_factor * v_ref
                lin_v = True

            if abs(self.angleDiff(des_angle - self.current[1])) > math.pi/2 \
                                and self.current[0] > 0.2:
                '''if goal point is downstream go towards it by floating with current.
                    This is done by mirroring the waypoint in a line perpendicular to
                    the current, through the boat. This point is used as the desired angle.
                    The reference velocity is chosen quite arbitrarily right now.
                    Better ways of calculating it is described below, but these needs to be tested'''

                des_angle_downstream = des_angle
                des_angle = self.angleDiff(2*self.current[1] - math.pi - des_angle)
                v_ref = -abs(0.5 * v_ref * math.cos(self.current[1] - des_angle_downstream))
                # For future: might be easier to just calculate the desired angle as desired angle + pi
                # and the just set vref as -vref. By doing it this way the boat will go backwards towards
                # the point. It would be easier to control the velocity by doing it like this.
                # As we do it now there are cases where we try to control the velocity in the boats "y-direction",
                # which is something we cannot do.

                # Another possibly better way of calculating v_ref, but needs to be tested!!
                # if point is to the side of the boat it will go slightly upstream before going down
                # v_ref = 0.5 * v_ref * math.cos(self.current[1] -
                #          np.sign(math.cos(des_angle)) * math.pi/3 - des_angle_downstream)

                self.debug_messages.action = 'going downstream'

            if lin_v:
                '''if we're close to the waypoint we limit the difference between the desired angle
                    and the current angle based on the distance to the point'''
                des_angle = np.clip(des_angle, self.current[1] - lin_factor*math.pi/2,
                                        self.current[1] + lin_factor*math.pi/2)

            e_ang = self.angleDiff(des_angle - ang_dir)
            self.debug_messages.des_ang = des_angle

        e_v = v_ref - vel_robot[0,0]    # error for rudder control

        rospy.logdebug("e_v " + str(e_v))
        rospy.logdebug('e_ang (in PI): ' + str(e_ang))
        u_rudder = self.rudder_control(e_ang)
        u_thrust = self.thrust_control(e_v)

        self.debug_messages.state_ref = self.state_ref
        self.debug_messages.state_asv = self.state_asv
        self.debug_messages.e_ang = e_ang
        self.debug_messages.e_v = e_v
        self.debug_messages.u_rudder = u_rudder
        self.debug_messages.u_thrust = u_thrust
        self.debug_messages.I_thrust = self.I_thrust
        self.debug_messages.I_rudder = self.I_rudder
        self.debug_messages.v_ref = v_ref
        self.debug_pub.publish(self.debug_messages)

        return u_thrust, u_rudder

    def thrust_control(self, e_v):
        """
        PI controller for the thrust control.

        Args:
            e_v (float): velocity error
        Returns:
            float: control output, bounded by -1000 and 1000
        """
        ##########################
        ### Control parameters ###
        MAX_THRUST = 1000
        MIN_THRUST = -1000
        '''controller on the form U(s) = K(1 + 1/(Ti*s))*E(s)'''

        # controller gains
        self.K_t = rospy.get_param('thrust/K', 400.0)
        self.Ti_t = rospy.get_param('thrust/Ti', 1.0)

        rospy.logdebug("THRUST:")
        rospy.logdebug("K: " + str(self.K_t))
        rospy.logdebug("Ti: " + str(self.Ti_t))
        ##########################

        # saturate output
        u = np.clip(-self.K_t*(e_v + self.h/self.Ti_t*self.I_thrust), \
                             MIN_THRUST, MAX_THRUST)

        # anti wind-up
        if u >= MIN_THRUST + 50  and u <= MAX_THRUST - 50:
            self.I_thrust = self.I_thrust + e_v

        rospy.logdebug("I_thrust: " + str(self.I_thrust))

        return u

    def rudder_control(self,e_ang):
        """
        PI controller for the servo control.

        Args:
            e_ang (float): angle error
        Returns:
            int: control output, bounded by 1196 and 1833
        """
        ##########################
        ### Control parameters ###
        MAX_RUDDER = 1833
        MIN_RUDDER = 1196

        '''controller on the form U(s) = K(1 + 1/(Ti*s))*E(s)'''

        # controller gains
        self.K_r = rospy.get_param('rudder/K', 150.0)
        self.Ti_r = rospy.get_param('rudder/Ti', 10.0)

        rospy.logdebug("RUDDER:")
        rospy.logdebug("K: " + str(self.K_r))
        rospy.logdebug("Ti: " + str(self.Ti_r))

        u = np.clip(-self.K_r*(e_ang + self.h/self.Ti_r*self.I_rudder)
                        + 1600.0, MIN_RUDDER, MAX_RUDDER)

        # anti wind-up
        if u >= MIN_RUDDER + 25  and u <= MAX_RUDDER - 25:
            self.I_rudder = self.I_rudder + e_ang

        rospy.logdebug("I_rudder: "+ str(self. I_rudder))

        return int(u)
