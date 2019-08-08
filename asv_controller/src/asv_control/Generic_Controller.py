#!/usr/bin/env python

import rospy
import math
import numpy as np
import time
from abc import abstractmethod


class Generic_Controller:
    
    def __init__(self, state_asv=None, state_ref=None, v_asv=None, \
            wayPoints=None, target_index=0, destReached=True, current=None, \
                controller=None):
        # need to instantiate member like this for mutable member        
        if state_asv == None:        
            self.state_asv = [0,0,0]
        if state_ref == None:
            self.state_ref = [0,0,0]
        if v_asv == None:
            self.v_asv = [0,0,0]
        if wayPoints == None:
            self.wayPoints = []
        if current == None:
            self.current = [0,0]
        
        # Control variables and inputs (generic to all controller)
        self.state_asv = [0,0,0]
        self.state_ref = [0,0,0]
        self.target_index = 0
        self.wayPoints = []
        self.destReached = True
        self.current = [0,0]
        
        # help pass data from the previous controller
        if controller != None:
            self.state_asv = controller.state_asv
            self.state_ref = controller.state_ref
            self.target_index = controller.target_index
            self.wayPoints = controller.target_index
            self.wayPoints = controller.wayPoints
            self.current = controller.current
        
        # Control Parameters (specific to class)

    def update_variable(self, s_asv, s_ref, v, target_i, wP, curr):
        '''get the update coming from the master '''
        self.state_asv = s_asv
        self.state_ref = s_ref
        self.v_asv = v
        self.target_index = target_i
        self.wayPoints = wP
        self.current = curr

    def destinationReached(self, destBool):
        self.destReached = destBool

    def angleDiff(self, angle):
        while angle > math.pi:
            angle = angle - 2 * math.pi
        while angle < -math.pi:
            angle = angle + 2 * math.pi
        return angle
    
    @abstractmethod
    def calc_control(self):
        pass

