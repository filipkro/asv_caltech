#!/usr/bin/env python

import serial
import rospy
from motor_control.msg import MotorCommand
from math import floor



def circ_test():
    ###################################
    ########### Test params ###########
    ANGLE = [1550, 1600, 1650, 1700, 1750, 1800]
    SPEED = [100, 200, 300, 400, 500, 600, 700, 800, 900, 1000]
    samp_rate = 10
    test_time = 20
    ###################################
    control_pub = rospy.Publisher('motor_controller\motor_cmd_reciever', MotorCommand, queue_size=1)
    msg = MotorCommand()
    rate = rospy.Rate(samp_rate)
    sim_time = len(ANGLE) * len(SPEED)
    prog = 0
    for i in range(len(ANGLE)):
        if rospy.is_shutdown():
            break;
        for j in range(len(SPEED)):
            print(str(floor(100*prog/sim_time)) + "%")
            prog += 1
            if rospy.is_shutdown():
                break;
            for k in range(test_time*samp_rate):
                msg.strboard = SPEED[j]
                msg.port = SPEED[j]
                msg.servo = ANGLE[i]
                control_pub.publish(msg)
                rate.sleep()
                if rospy.is_shutdown():
                    break;

    msg.strboard = 0.0
    msg.port = 0.0
    msg.servo = 1500
    control_pub.publish(msg)

def lin_test():
    ###################################
    ########### Test params ###########
    '''what is reasonable? Maybe 0-2000'''
    SPEED = [100, 200, 300, 400, 500, 600, 700, 800, 900, 1000]
    samp_rate = 10
    test_time = 20
    RUDDER_ANG = 1500
    '''or whatever is straight'''
    ###################################
    control_pub = rospy.Publisher('motor_controller\motor_cmd_reciever', MotorCommand, queue_size=1)
    msg = MotorCommand()
    rate = rospy.Rate(samp_rate)
    sim_time = len(SPEED)
    prog = 0
    for i in range(len(SPEED)):
        if rospy.is_shutdown():
            break;
        print(str(floor(100*prog/sim_time)) + "%")
        prog += 1
        for k in range(test_time*samp_rate):
                '''how should rpms be translated to motorCommand?'''
                msg.strboard = SPEED[i]
                msg.port = SPEED[i]
                msg.servo = RUDDER_ANG
                control_pub.publish(msg)

                rate.sleep()
                if rospy.is_shutdown():
                    break;

    msg.strboard = 0.0
    msg.port = 0.0
    msg.servo = RUDDER_ANG
    control_pub.publish(msg)

def main():
    rospy.init_node('data_collect')

#########  Which data to collect?  #########
    #lin_test()
    circ_test()


if __name__ == '__main__':
    main()
