#!/usr/bin/env python

import serial
import rospy
import math
from motor_control.msg import MotorCommand
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from roboteq_msgs.msg import Command
from gps_reader.msg import GPS_WayPoints
from sensor_msgs.msg import Joy
import numpy as np

strboard_ser = None
port_ser = None

port_command = Command()
star_command = Command()
servo_command = 1600

servo_pub = rospy.Publisher('servo_cmd', UInt32, queue_size=10) 
port_pub = rospy.Publisher('starboard_driver/cmd', Command, queue_size=10)
star_pub = rospy.Publisher('port_driver/cmd', Command, queue_size=10)
heading_pub = rospy.Publisher('heading', Float32, queue_size=10)
#port_pub = rospy.Publisher('roboteq_driver/port/cmd', Command, queue_size=10)
#star_pub = rospy.Publisher('roboteq_driver/star/cmd', Command, queue_size=10)

# note that the servo is on the arduino, connected via rosserial
# therefore we need to publish to this topic for servo control.

def setup_serial():
    '''setup serial port for the motor '''
    global port_ser, strboard_ser
    port_ser = serial.Serial('/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FT8VWDWP-if00-port0', 115200)
    strboard_ser = serial.Serial('/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FT8VW9AR-if00-port0', 115200)
    # we're using the roboteq controller, therefore no longer need to call our own
    # jk we messed up the controller, now back to the basic

def send_cmd_callback(data):
    '''send command to motor everytime we recieved a callback'''
    # motor value is between -1000 to 1000
    update_cmd(data.port, data.strboard, data.servo)

def teleop_callback(data):
    '''send command to motor with teleop.'''
    # TODO: we're randomly mapping speed to motor value
    # when we have properly calibrate we should get rid
    # of this function

    forward = min(1000, max(-1000, data.linear.x * 100))
    turn = min(1800, max(1200, -data.angular.z*200 + 1600))

    # send serial command     # publish to servo
    update_cmd(forward, forward, turn)

def joy_callback(data):
    '''send command to motor based on joystick'''
    leftStick_y = data.axes[1]
    leftStick_x = data.axes[0]
    rightStick_y = data.axes[4]

    port_cmd = -leftStick_y * 1000
    starboard_cmd = -rightStick_y * 1000
    servo_cmd = leftStick_x * (1834 - 1195)/2 + 1600

    servo_cmd = np.clip(servo_cmd, 1195, 1834)
    
    update_cmd(port_cmd, starboard_cmd, servo_cmd)

def update_cmd(port, starboard, servo):
    '''update the command to be sent to the motors'''
    global port_command, star_command, servo_command

    # For simulation, publish to gazebo stuff (not done yet)
    if rospy.get_param('/motor_control/sim', False):
        print('Port, Starboard, Servo', port, starboard, servo)
    else:
        servo_command = servo
        # convert from rpm to rad/s
        # port = -port/ 60 * 2* math.pi
        # port_command.mode = 2
        # port_command.setpoint = port

        # starboard = -starboard / 60 * 2* math.pi
        # star_command.mode = 2
        # star_command.setpoint = starboard

        port_command = '!G 1 %d' % port
        starboard_command = '!G 1 %d' % starboard
        port_ser.write(port_command.encode() + b'\r\n')
        strboard_ser.write(starboard_command + b'\r\n')
        
def imu_callback(msg):
    offet = float(rospy.get_param('/motor_control/compass_offset', 0.0))
    theta = angleDiff(msg.data[8] - 109.0/180.0 * math.pi)
    heading_pub.publish(theta)

def test_callback(msg):
    print("I GOT THE MESSAGE")

def angleDiff(angle):
    while angle > math.pi:
        angle = angle - 2 * math.pi
    while angle < -math.pi:
        angle = angle + 2 * math.pi

    return angle

def set_servo_straight():
    servo_pub.publish(1600)
    print('shutting down')

def main():
    sim = rospy.get_param('/motor_control/sim', False)

    setup_serial()

    rospy.init_node('motor_controller')
    rospy.Subscriber('motor_controller/motor_cmd_reciever', MotorCommand, send_cmd_callback)
    rospy.Subscriber('cmd_vel', Twist, teleop_callback)
    rospy.Subscriber('imu', Float32MultiArray, imu_callback)
    rospy.Subscriber('joy', Joy, joy_callback)

    rospy.on_shutdown(set_servo_straight)

    rate = rospy.Rate(50) # 50hz
    while not rospy.is_shutdown():
        # publish servo
        servo_pub.publish(servo_command)

        # publish motor
        # port_pub.publish(port_command)
        # star_pub.publish(star_command)

        rate.sleep()
    
