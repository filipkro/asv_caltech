#!/usr/bin/env python
import rospy
import math
import numpy as np
from Generic_Controller import Generic_Controller

# inherited from the Generic Controller class
# members:
    # state_asv, state_ref, current, v_asv, wayPoints, target_index,
    # destReached
# Functions:
    # destinationReached
    # update_variable
    # angleDiff

# note: for transect control, index can only go between 0 and 1

class Transect_controller(Generic_Controller):
    def __init__(self, controller=None):
        Generic_Controller.__init__(self, controller=controller)

        # member specific to transect controller
        self.transect_p1 = None
        self.transect_p2 = None
        self.last_point = None
        self.v_update_count = 0 # update counters to slow down the update
        self.ang_update_count = 0
        self.last_u_nom = 0   # previous command
        self.last_v_des = 0   # previous desired speed
        self.last_ang_des = 0 # previous deisred angle

        # parameters specific to transect controller
        self.K_v = rospy.get_param('/transect/K_v', 0.5) # vertical speed gain
        self.K_t = rospy.get_param('transect_thrust/K', 10.0)
        self.K_latAng = rospy.get_param('/transect/K_latAng', 0.5) # lateral speed to angle
        self.Kp_turn = rospy.get_param('/transect/Kp_turn', 200.0) # turning gain for heading
        self.v_x_des = rospy.get_param('/transect/speed_ref', 0.5)
        self.DIST_THRESHOLD = rospy.get_param('/transect/dist_thres', 1.0) #unused
        self.ang_update_rate = rospy.get_param('/transect/ang_update_rate', 1)
        self.v_update_rate = rospy.get_param('/transect/v_update_rate', 1)

    def calc_control(self):
        self.v_x_des = rospy.get_param('/transect/speed_ref', 0.5)
        self.DIST_THRESHOLD = rospy.get_param('/transect/dist_thres', 1.0)

        if (self.target_index == 1):
            # simply switch way points between the first two
            self.last_point = [self.wayPoints[0].x, self.wayPoints[0].y, 0.0]
            self.state_ref = [self.wayPoints[1].x, self.wayPoints[1].y, 0.0]
        else:
            self.last_point = [self.wayPoints[1].x, self.wayPoints[1].y, 0.0]
            self.state_ref = [self.wayPoints[0].x, self.wayPoints[0].y, 0.0]

        rospy.logdebug('Last point ' + str(self.last_point))
        rospy.logdebug('Point now ' + str(self.state_ref))
        print('state ref', self.state_ref)
        u_rudder = 1600
        self.v_update_rate = rospy.get_param('/transect/v_update_rate', 1)
        self.ang_update_rate = rospy.get_param('/transect/ang_update_rate', 1)
        print('v cnt', self.v_update_count)
        print('v rate', self.v_update_rate)
        if self.v_update_count >= self.v_update_rate:
            v_vert, u_nom = self.vertical_speed_control()
            self.last_v_des = v_vert
            self.last_u_nom = u_nom
            self.v_update_count = 0
            print('HEREERE', u_nom)
        else:
            self.v_update_count += 1
            v_vert = self.last_v_des
            u_nom = self.last_u_nom

            print('not updating', u_nom)

        print('v cnt after', self.v_update_count)
        print('v rate after', self.v_update_rate)

        if self.ang_update_count >= self.ang_update_rate:
            ang_des = self.calc_lateral_ang()
            self.last_ang_des = ang_des
            self.ang_update_count = 0
        else:
            self.ang_update_count += 1
            ang_des = self.last_ang_des

        u_rudder = self.heading_control(ang_des)
        u_nom = -np.clip(u_nom, -1000.0, 1000.0)
        return u_nom, u_rudder

    def vertical_speed_control(self):
        '''calculate current velocity toward the waypoint, increase or decrease u_nom'''
        global K_v, last_point, v_asv, state_asv
        self.K_v = rospy.get_param('/transect/K_v', 50.0)
        self.K_t = rospy.get_param('/transect_thrust/K', 10.0)

        # Calculate drift away from the line
        cur_point = self.last_point
        next_point = self.state_ref

        line_angle = math.atan2((self.state_ref[1] - self.last_point[1]), \
                            (self.state_ref[0] - self.last_point[0]))
        position_angle = math.atan2((self.state_asv[1] - self.last_point[1]), \
                             (self.state_asv[0] - self.last_point[0]))

        # calculate position from the line
        pos_ang_from_line = self.angleDiff(position_angle - line_angle) # position vector from line
        dist = math.sqrt((self.state_asv[1] - self.last_point[1])**2 + \
                                  (self.state_asv[0] - self.last_point[0])**2 )
        drift_distance = dist * math.sin(pos_ang_from_line)

        # calculate velocity away from the line
        v_ang_from_line = self.angleDiff(self.state_asv[2] - line_angle) # v_vector and line
        v_course = math.sqrt(self.v_asv[0]**2 + self.v_asv[1]**2)
        drift_v = v_course * math.sin(v_ang_from_line)

        # heading vector from line
        heading_from_line = self.angleDiff(self.state_asv[2] - line_angle)

        # thrust direction
        thrust_dir = heading_from_line/abs(heading_from_line)

        # adding an integral term to remove error (ignore for now)
        # drift_error_integral = self.drift_error_integral + drift_distance * self.dt
        v_correct = -drift_distance * self.K_v #- self.drift_error_integral * self.K_vi
        nominal_u = rospy.get_param('/transect/u_nominal', 100)
        u_nom = (v_correct - drift_v) * thrust_dir * self.K_t #+ nominal_u # or 10

        rospy.logdebug("Drift dist " + str(drift_distance))
        rospy.logdebug("Drift v " + str(drift_v))
        rospy.logdebug("V correct " + str(v_correct))
        rospy.logdebug("u_nom " + str(u_nom))
        rospy.logdebug("thrust_dir " + str(thrust_dir))


        return v_correct, u_nom

    def calc_lateral_ang(self):
        '''Adjust the ASV angle according to the desired transect (x) speed'''
        self.K_latAng = rospy.get_param('/transect/K_latAng', 0.5)

        # Calculate drift away from the line
        cur_point = self.last_point
        next_point = self.state_ref

        line_angle = math.atan2((self.state_ref[1] - self.last_point[1]), \
                                    (self.state_ref[0] - self.last_point[0]))
        v_ang_from_line = self.angleDiff(self.v_asv[2] - line_angle)

        v_course = math.sqrt(self.v_asv[0]**2 + self.v_asv[1]**2)
        v_x = v_course * math.cos(v_ang_from_line) # this v_x is along the line

        # Calculate heading difference between robot and the line
        heading_from_line = self.angleDiff(self.state_asv[2] - line_angle)

        if heading_from_line < 0:
            des_line_heading = self.angleDiff(-(v_x - self.v_x_des) * \
                                            self.K_latAng + heading_from_line)
        else:
            des_line_heading = self.angleDiff((v_x - self.v_x_des) * \
                                         self.K_latAng + heading_from_line)


        new_ang = self.angleDiff(des_line_heading + line_angle)
        rospy.logdebug('V_x ' + str(v_x))
        rospy.logdebug('V_x error' + str(v_x - self.v_x_des))
        rospy.logdebug('line angle ' + str(line_angle))
        rospy.logdebug('heading ' + str(self.state_asv[2]))
        rospy.logdebug('heading from lne ' + str(heading_from_line))
        rospy.logdebug('des heading from line ' + str(des_line_heading))
        rospy.logdebug("Ang des " + str(new_ang))
        # print("V_x Difference ", v_x - v_x_des)
        # print("Desired Angle ", new_ang)
        # print("Current Angle ", self.state_est.theta)
        return new_ang

    def heading_control(self, heading_des):
        '''Orient the robot heading by turning the rudder'''
        self.Kp_turn = rospy.get_param('/transect/Kp_turn', 200)
        u_rudder = -self.angleDiff(heading_des - self.state_asv[2]) \
                                                            * self.Kp_turn
        u_rudder = int(u_rudder + 1600)
        if (u_rudder > 1834):
            u_rudder = 1834
        elif (u_rudder < 1195):
            u_rudder = 1195
        rospy.logdebug("rudder " +  str(u_rudder))
        return u_rudder
