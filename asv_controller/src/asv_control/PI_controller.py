#!/usr/bin/env python

import rospy
import math
import numpy as np
import time
from Generic_Controller import Generic_Controller

# inherited from the Generic Controller class
# members:
    # state_asv, state_ref, current, v_asv, wayPoints, target_index,
    # destReached
# Functions:
    # destinationReached
    # update_variable
    # angleDiff

class PI_controller(Generic_Controller):

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
        self.v_robotX = 0.0
        self.downstream = False


    def d2t(self):
        return math.sqrt((self.state_asv[0] - self.state_ref[0])**2 +
                        (self.state_asv[1] - self.state_ref[1])**2)

    # TODO: maybe make this an abstract method for generic controller
    def calc_control(self):
        rospy.logdebug("Desired wp: " + str(self.state_ref))
        rospy.logdebug("vel(x,y): " + str(self.v_asv))

        # update the param for live change from the user
        self.V_REF = rospy.get_param('v_ref', 0.5)
        self.VEL_THRESHOLD = rospy.get_param('/v_threshold', 0.2)
        v_ref = self.V_REF

        self.downstream = False

        '''transform velocities to robots coordinate system'''
        rot = np.array([[np.cos(self.state_asv[2]), np.sin(self.state_asv[2])],
            [-np.sin(self.state_asv[2]), np.cos(self.state_asv[2])]])
        vel_unrot = np.array([[self.v_asv[0]],[self.v_asv[1]]])
        vel_robot = np.matmul(rot, vel_unrot)
        self.vel_robotX = vel_robot[0]

        if rospy.get_param('/I/reset', False): #or not rospy.get_param('/motor_control/sim', True):
            self.I_rudder = 0.0
            self.I_thrust = 0.0

#        print('state ref', self.state_ref)
#        print('state asv', self.state_asv)
        # print('current angle', self.current[1])
        # print('heading angle', self.state_asv[2])
        # print('current vel', self.current[0])
        # print('vel boat', self.v_asv)

        ####### Changes to speed to target ############### ##NOT WORKING
        # v = math.sqrt(v_asv[0]**2 + v_asv[1]**2)
        # ang_offset = math.atan2(state_ref[1] - state_asv[1], state_ref[0] - state_asv[0])
        # vel_2_target_x = v* math.cos(angleDiff(v_asv[2] - ang_offset))
        # vel_2_target_y = v* math.sin(angleDiff(v_asv[2] - ang_offset))
        # vel_2_target = np.matrix([[ v* math.cos(angleDiff(v_asv[2] - ang_offset))], \
        #                             [v* math.sin(angleDiff(v_asv[2] - ang_offset)) ]])
        # vel_robot = vel_2_target
        # print(vel_robot)
        ##################################################

        lin_v = False

        self.dist_threshold = rospy.get_param('/dist_threshold', 0.5)

        '''evaluate rotation of robot compared to current'''
        e_heading = self.angleDiff(self.current[1] - self.state_asv[2])
        rospy.logdebug(e_heading)
        ang_dir = self.state_asv[2]

        v = math.sqrt(self.v_asv[0]**2 + self.v_asv[1]**2)

        if abs(e_heading) > math.pi/3 and self.current[0] > 0.1:
            rospy.logdebug('FORSTA')
            # v_ref = vel_robot[0,0] + 0.5
            '''turn robot back towards current if it points to much downward'''
            e_ang = e_heading

        elif self.d2t() < 0.3:
            e_ang = e_heading
            v_ref = 0.0

        else:

        # elif self.destReached:
        #     rospy.logdebug('ANDRA')
        #     '''if completed turn boat towards current, keep velocity at 0.0
        #         reason not to have this if statement
        #         first is thrust is needed to rotate boat'''
        #     e_ang = e_heading
        #     rospy.logdebug(e_heading)
        #     rospy.logdebug(self.current)
        #     v_ref = 0.0
        #else:
            rospy.logdebug('TREDJE')
            '''desired angle and velocity of robot'''
            des_angle = math.atan2(self.state_ref[1] - self.state_asv[1], \
                            self.state_ref[0] - self.state_asv[0])
            v = math.sqrt(self.v_asv[0]**2 + self.v_asv[1]**2)

            '''desicion on which feedback to use for angle'''
            if v < self.VEL_THRESHOLD or vel_robot[0,0] < 0.0:
                '''if moving slowly or backwards, use heading as feedback'''
                ang_dir = self.state_asv[2]
            else:
                '''else use GPS'''
                ang_dir = self.v_asv[2]

            if self.d2t() < rospy.get_param('/d2t', 1.5):
                lin_factor = (self.d2t() - self.dist_threshold)/rospy.get_param('/d2t', 1.5)
                v_ref = lin_factor * v_ref
                lin_v = True
#                print('in lin', lin_factor)

            # if self.transect and v > self.VEL_THRESHOLD:
            #     # rot = np.array([[np.cos(self.theta_p), np.sin(self.theta_p)],
            #     #     [-np.sin(self.theta_p), np.cos(self.theta_p)]])
            #     # vel_unrot = np.array([[self.v_asv[0]],[self.v_asv[1]]])
            #     # vel_robot = -np.matmul(rot, vel_unrot)
            #
            #     # '''transform velocities to robots coordinate system'''
            #     rot = np.array([[np.cos(self.state_asv[2]), np.sin(self.state_asv[2])],
            #         [-np.sin(self.state_asv[2]), np.cos(self.state_asv[2])]])
            #     vel_unrot = np.array([[self.v_asv[0]],[self.v_asv[1]]])
            #     vel_robot = np.matmul(rot, vel_unrot)
            #     self.vel_robotX = vel_robot[0]
            #
            #     self.prev_Kt = rospy.get_param('/thrust/K', 400.0)
            #     self.prev_Kr = rospy.get_param('/rudder/K', 150.0)
            #     rospy.set_param('thrust/K', 100.0)
            #     rospy.set_param('/rudder/K', 50.0)


            if abs(self.angleDiff(des_angle - self.current[1])) > math.pi/2 \
                                and self.current[0] > 0.2:
                '''if goal point is downstream go towards it by floating with current'''
                des_angle_downstream = des_angle
                des_angle = self.angleDiff(2*self.current[1] - math.pi - des_angle)
                if lin_v:
                    des_angle = np.clip(des_angle, self.current[1] - lin_factor*math.pi/2,
                                            self.current[1] + lin_factor*math.pi/2)
                # e_ang = self.angleDiff(math.pi - des_angle + ang_dir)
                # v_ref = -v_ref/2
                v_ref = -abs(0.5 * v_ref * math.cos(self.current[1] - des_angle_downstream))
#                print('v_ref', v_ref)
                # rospy.logdebug("angle error " + str(e_ang))
                # self.downstream = True
                '''transform velocities to error coordinate frame'''
                # rot = np.array([[np.cos(des_angle_downstream), np.sin(des_angle_downstream)],
                #     [-np.sin(des_angle_downstream), np.cos(des_angle_downstream)]])
                # vel_unrot = np.array([[self.v_asv[0]],[self.v_asv[1]]])
                # vel_robot = -np.matmul(rot, vel_unrot)



            e_ang = self.angleDiff(des_angle - ang_dir)


                #
                # v

                # e_ang = self.angleDiff(self.theta_p - self.v_asv[2])




            # elif abs(self.angleDiff(des_angle - self.current[1])) > math.pi/4 \
            #             and self.current[0] > 0.2 and  \
            #             self.d2t() < 3 * rospy.get_param('/dist_threshold', 1.0):
            #     v_ref = v_ref/2
            #     e_ang = self.angleDiff(des_angle - ang_dir)
            #     self.downstream = False

            # else:
            #     e_ang = self.angleDiff(des_angle - ang_dir)
            #     self.downstream = False

            # if abs(self.angleDiff(self.current[1] + math.pi - self.state_asv[2])) < 0.05 \
            #             and abs(self.current[0] - self.V_REF) < 0.1:
            #     v_ref -= 0.25


        # rot = np.array([[np.cos(ang_dir), np.sin(ang_dir)],
        #     [-np.sin(ang_dir), np.cos(ang_dir)]])
        # vel_unrot = np.array([[self.v_asv[0]],[self.v_asv[1]]])
        # vel_robot = np.matmul(rot, vel_unrot)
        # self.vel_robotX = vel_robot[0]
        # print('lin_v', lin_v)
        # print('vel robot', vel_robot[0,0])
        e_v = v_ref - vel_robot[0,0]
        # print('vel error', e_v)
        # print('ang error',e_ang)
        rospy.logdebug("e_v " + str(e_v))
        rospy.logdebug('e_ang (in PI): ' + str(e_ang))
        u_rudder = self.rudder_control(e_ang)
        u_thrust = self.thrust_control(e_v)
        # if self.transect and v > self.VEL_THRESHOLD:
        #     rospy.set_param('thrust/K', self.prev_Kt)
        #     rospy.set_param('/rudder/K', self.prev_Kr)
        ### extension for using velocity to target
        # check which direction we're point at #
        # des_angle = math.atan2(state_ref[1] - state_asv[1], state_ref[0] - state_asv[0])

        # if ( abs(state_asv[2] - des_angle) > math.pi/2):
        #     u_rudder = u_rudder * -1

        return u_thrust, u_rudder

    def thrust_control(self, e_v):
        ''' PI controller. Controls thrust to keep contant velocity.
        Should work in different currents? '''
        ##########################
        ### Control parameters ###
        MAX_THRUST = 1000
        MIN_THRUST = -1000
        '''controller on the form U(s) = K(1 + 1/(Ti*s))*E(s)'''

        self.K_t = rospy.get_param('thrust/K', 400.0)
        self.Ti_t = rospy.get_param('thrust/Ti', 1.0)

        rospy.logdebug("THRUST:")
        rospy.logdebug("K: " + str(self.K_t))
        rospy.logdebug("Ti: " + str(self.Ti_t))
        ##########################

        u = np.clip(-self.K_t*(e_v + self.h/self.Ti_t*self.I_thrust), \
                             MIN_THRUST, MAX_THRUST)
        if u >= MIN_THRUST + 50  and u <= MAX_THRUST - 50:
            self.I_thrust = self.I_thrust + e_v

        rospy.logdebug("I_thrust: " + str(self.I_thrust))

        return u

    def rudder_control(self,e_ang):
        ##########################
        ### Control parameters ###
        MAX_RUDDER = 1833
        MIN_RUDDER = 1196

        '''controller on the form U(s) = K(1 + 1/(Ti*s))*E(s)'''

        self.K_r = rospy.get_param('rudder/K', 150.0)
        self.Ti_r = rospy.get_param('rudder/Ti', 10.0)
        ##########################
        rospy.logdebug("RUDDER:")
        rospy.logdebug("K: " + str(self.K_r))
        rospy.logdebug("Ti: " + str(self.Ti_r))


        rospy.logdebug('ang_course ' + str(self.v_asv[2]))
        rospy.logdebug("theta " + str(self.state_asv[2]))
        rospy.logdebug("e ang " + str(e_ang))

        u = np.clip(-self.K_r*(e_ang + self.h/self.Ti_r*self.I_rudder)
                        + 1600.0, MIN_RUDDER, MAX_RUDDER)
        if u >= MIN_RUDDER + 50  and u <= MAX_RUDDER - 50:
            self.I_rudder = self.I_rudder + e_ang

        rospy.logdebug("I_rudder: "+ str(self. I_rudder))
        return int(u)
        '''set Tr to 1 and move K/Ti to calculation of u??'''
