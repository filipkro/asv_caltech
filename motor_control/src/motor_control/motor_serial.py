#!/usr/bin/env python

import serial
import rospy
import math
from motor_control.msg import MotorCommand
from motor_control.msg import Arduino_imu
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from roboteq_msgs.msg import Command
from gps_reader.msg import GPS_WayPoints
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
import numpy as np
import tf

"""Motor Serial
This module contains implementation for the motor_control_node.
It has multiple functions:
    1. Talk to STARBOARD and PORT motor driver
    2. Control the SERVO through the arduino node.
    3. Extract HEADING data from the arduino node
    4. Recieves MotorCommand from the asv_controllers
    5. Takes in joy data from a joy stick and converts
        those value to motor command (optional)
    6. Takes in key board teleop (optional)

Published topics:
    /servo_cmd
    /heading

Subscribed topics:
    /imu (Arduino_imu)
    /motor_controller/motor_cmd_reciever  (MotorCommand)
    /cmd_vel (Twist)
    /joy (Joy)
    /os1/imu (Imu) 

Example:
    If servo control and imu is necessary, you have to initate the Arduino node first

        rosrun rosserial_python serial_node.py _port:="/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_FFFFFFFFFFFF515B2503-if00"

    Running this will establish connection with the motor driver, overriding the RC link
    
        rosrun motor_control motor_control_node.py
    
    Run this to use the joy stick. Rememebr to plug in the PS4 controller

        rosrun joy joy_node

    To get the lidar imu data, see the imu_filter_madgwick section in 
        asv_controller/launch/smart_cntrl.launch


Notes:
    1. For more detail on the actual drivers, they are roboteq
        MBL1660 brushless motor driver. Specific serial ASCII commands
        can be found in the documentation.
    2. Why are we republishig Heading data here? 
        This might be a bit unintuitive... why not just have other 
        nodes subscribe to the /imu topic directly? For some reason,
        the controller node won't read the imu topic, so we have a 
        temprary work around by publishing the heading here to the 
        /heading topic. Please resolve this if there is solution. 
    3. Prior to using pyserial, we used a motor driver called roboteq_driver.
       Hence there are commented out line for publish topic to /roboteq_driver
       The drivers can extract voltage level from the battery, however
       they're not super easy to use. Hence we left it out. 
    4. We used two custom defined message. MotorCommand, and Arduino_imu,
       the latter is essentially a time stamped float32

"""

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
    # jk we messed up the controller, now back to the pyserial

def send_cmd_callback(data):
    '''send command to motor everytime we recieved a callback
    
    Args:
        data (MotorCommand): msg that contains motor command data'''
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
    '''send command to motor based on joystick

    Args:
        data (joy): joy stick messages
    '''
    leftStick_y = data.axes[1]
    leftStick_x = data.axes[0]
    rightStick_y = data.axes[4]

    port_cmd = -leftStick_y * 1000
    starboard_cmd = -rightStick_y * 1000
    servo_cmd = leftStick_x * (1834 - 1195)/2 + 1600

    servo_cmd = np.clip(servo_cmd, 1195, 1834)
    
    update_cmd(port_cmd, starboard_cmd, servo_cmd)

def update_cmd(port, starboard, servo):
    '''update the command to be sent to the motors
    
    Args:
        port (float): motor commabd value between -1000 and 1000
        starboard (float): motor command value between -1000 and 1000
        servo (float): servo command value
    '''
    global port_command, star_command, servo_command

    # For simulation, publish to gazebo stuff (not done yet)
    if rospy.get_param('/motor_control/sim', False):
        rospy.logdebug('Port, Starboard, Servo' + str(port) + str(starboard) + str(servo))
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
    """ Subscribe to imu topics published by the arduino and extract the heading.
    Compass offset is applied here through the param
        /motor_control/compass_offset
    
    Args:
        msg (Arduino_imu): time stamped Float32MultiArray 
                [acc_x, acc_y, acc_z, gx, gy, gz, roll, pitch, yaw]
    """
    offet = float(rospy.get_param('/motor_control/compass_offset', 109.0))
    theta = angleDiff(msg.data[8] - offet / 180.0 * math.pi)
    heading_pub.publish(theta)

def imu_lidar_callback(msg):
    '''extract the lidar imu data on the os1_lidar. See example on how to run this
    
    Args:
        msg (Imu): imu data os1_lidar
    '''
    offet = float(rospy.get_param('/lidar_imu_offset', 0.0))
    quat = msg.orientation
    quaternion = [quat.x, quat.y, quat.z, quat.w]
    euler =  tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    rospy.logdebug("os1 imu" + str(roll) + str(pitch) + str(yaw))
    

def test_callback(msg):
    print("I GOT THE MESSAGE")

def angleDiff(angle):
    '''Bound an angle between -pi and +pi in radians
    
    Args:
        angle (float): angle to be bounded
    Returns:
        float: angle that is bounded
    '''
    while angle > math.pi:
        angle = angle - 2 * math.pi
    while angle < -math.pi:
        angle = angle + 2 * math.pi

    return angle

def set_servo_straight():
    ''' 
        On ROS shutdwon, this will publish a message to the arduino node
        to set the servo straight. There is also another fail safe in the 
        arduino code itself.
    '''
    servo_pub.publish(1600)
    print('shutting down')

def main():
    sim = rospy.get_param('/motor_control/sim', False)

    setup_serial()

    rospy.init_node('motor_controller')
    rospy.Subscriber('motor_controller/motor_cmd_reciever', MotorCommand, send_cmd_callback)
    rospy.Subscriber('cmd_vel', Twist, teleop_callback)
    rospy.Subscriber('imu', Arduino_imu, imu_callback)
    rospy.Subscriber('joy', Joy, joy_callback)
    rospy.Subscriber('os1/imu', Imu, imu_lidar_callback)

    rospy.on_shutdown(set_servo_straight)

    rate = rospy.Rate(50) # 50hz
    while not rospy.is_shutdown():
        # publish servo
        servo_pub.publish(servo_command)

        # publish motor
        # port_pub.publish(port_command)
        # star_pub.publish(star_command)

        rate.sleep()
    
