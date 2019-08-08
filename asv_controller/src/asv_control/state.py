#!/usr/bin/env python

#state.py
#using this guy's design 
# https://dev.to/karn/building-a-simple-state-machine-in-python


class State(object):
    """
    We define a state object which provides some utility functions for the
    individual states within the state machine.
    """

    def __init__(self):
        print 'Processing current state:', str(self)
        self.controller = None # each state has their own controller
        self.ranges = []
        self.lidar_inc = 0.0083

    def __repr__(self):
        """
        Leverages the __str__ method to describe the State.
        """
        return self.__str__()

    def __str__(self):
        """
        Returns the name of the State.
        """
        return self.__class__.__name__

    def on_event(self, event):
        """
        Handle events that are delegated to this State.
        """
        pass

    def update_controller_var(self, state_asv, state_ref, v_asv, \
                                        target_index, wayPoints, current):
        '''
        Update the variable for the controller
        '''
        self.controller.update_variable(state_asv, state_ref, v_asv, \
                                        target_index, wayPoints, current)

    def update_lidar(self, lidar_ranges, lidar_inc):
        self.ranges = lidar_ranges
        self.lidar_inc = lidar_inc

    def calc_control(self):
        '''
        Use the controller and return controls appropriate for this state
        ''' 
        print('implement this!')
        return 0,0

