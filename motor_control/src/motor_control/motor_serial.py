#!/usr/bin/env python

import serial
import rospy
from motor_control.msg import MotorCommand
from geometry_msgs.msg import Twist


strboard_ser = None
port_ser = None

def setup_serial():
    '''setup serial port for the motor '''
    global port_ser, strboard_ser
    port_ser = serial.Serial('/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FT8VWDWP-if00-port0', 115200)
    strboard_ser = serial.Serial('/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FT8VW9AR-if00-port0', 115200)

def send_cmd_callback(data):
    '''send command to motor everytime we recieved a callback'''
    # motor value is between -1000 to 1000

def teleop_callback(data):
    '''send command to motor with teleop'''

    # TODO: calibrate the speed to pwm
    forward = min(1000, max(-1000, data.linear.x * 100))
    turn = min(1800, max(1200, data.angular.z*200 + 1600))

    update_cmd(forward, forward, turn)

def update_cmd(port, starboard, servo):
    '''send command to motor'''
    global port_ser, strboard_ser

    if rospy.get_param('/sim'):
        print('Port, Starboard, Servo', port, starboard, servo)
    else:
        port_command = '!G 1 %d' % port
        starboard_command = '!G 1 %d' % starboard
        port_ser.write(port_command.encode() + + b'\r\n')
        strboard_ser.write(starboard_command + b'\r\n')

def main():
    sim = rospy.get_param('/sim')

    if sim == True:
        rospy.loginfo('YEET')
    else:
        setup_serial()

    rospy.init_node('motor_controller')
    rospy.Subscriber('motor_cmd_reciever', MotorCommand, send_cmd_callback)
    rospy.Subscriber('cmd_vel', Twist, teleop_callback)
    rospy.spin()
