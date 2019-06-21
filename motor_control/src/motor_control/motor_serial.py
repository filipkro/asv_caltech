#!/usr/bin/env python

import serial
import rospy
from motor_control.msg import MotorCommand
strboard_ser = None
port_ser = None

def setup_serial():
    '''setup serial port for the motor '''
    global port_ser, strboard_ser
    port_ser = serial.Serial('/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FT8VWDWP-if00-port0', 115200)
    strboard_ser = serial.Serial('/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FT8VW9AR-if00-port0', 115200)

def send_cmd_callaback(data):
    '''send command to motor everytime we recieved a callback'''
    

def main():
    rospy.init_node('motor_controller')
    rospy.Subscriber('motor_cmd_reciever', MotorCommand, send_cmd_callback)

    rospy.spin()
