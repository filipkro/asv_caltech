#!/usr/bin/env python

import rospy
from gps_reader.msg import GPS_data
from motor_control.msg import MotorCommand
import math
import numpy as np

pub_gps = rospy.Publisher('/GPS/xy_coord', GPS_data, queue_size=10)
gps_message = GPS_data()

x = 0.0
y = 0.0



omega = 0.0
v = 0.0
theta = 0.0


def angleDiff(angle):
    while angle > math.pi:
        angle = angle - 2 * math.pi
    while angle < -math.pi:
        angle = angle + 2 * math.pi

    return angle


#def update_state(self, state, uR, uL, rudder):
def update_state(msg):
    ''' for simulation '''
    global pub_gps, gps_message, x, y, v, omega, theta

    #uR = -uR/ 100
    #uL = -uL/ 100

    uR = msg.strboard/400.0
    uL = msg.port/400.0
    rudder = msg.servo

    current_v = 0.0
    current_ang = 0.0

    # Rudder Angle
    rudder_rate = (1894.0 - 1195.0)/90.0
    rudder_ang = (rudder - 1515.0) / rudder_rate
    rudder_ang = -rudder_ang / 180.0 * math.pi

    b_l = 4.0 # sim linear drag
    b_r = 2.5 # sim rotational drag
    I_zz = 1.0 # sim moment of inertia
    m = 5.0 # sim mass
    robot_radius = 0.5
    x_cg = 0.9
    w = 20.0

    dt = 0.2

     # Rudder Angle
   #  b_l = 5 # sim linear drag
   #  b_r = 2.5 # sim rotational drag
   #  I_zz = 50 # sim moment of inertia
   #  m = 5 # sim mass
   #  robot_radius = 0.5

   # # update state
   #  a = (uR + uL)/m - b_l/m * v
   #  ang_acc = -b_r / I_zz * omega  + x_cg * rudder_ang * (uR + uL) * math.sin(rudder_ang) # + 1/I_zz * 2 * robot_radius * (uL - uR)

   # update river current
    # current_v = 2 * state.x / w * (2*state.x / w - 2) * state.current_v

#    current_v = state.current_v

   # update state
    a = (uR + uL)/m - b_l/m * v
    ang_acc = -b_r / I_zz * omega  +  (uR + uL) * math.sin(rudder_ang) * x_cg / I_zz

    v = v + a * dt
    omega = omega + ang_acc * dt

    robot_vx = v * math.cos(theta) + current_v * math.cos(current_ang)
    robot_vy = v * math.sin(theta) + current_v * math.sin(current_ang)
    v_robot = np.array([robot_vx, robot_vy])

    ang_course = math.atan2(v_robot[1], v_robot[0])
    v_course = math.sqrt(np.dot(v_robot,v_robot))

    omega = min(max(omega, -1.0), 1.0)

    # update position
    x = x + v*math.cos(angleDiff(theta)) * dt + current_v * math.cos(angleDiff(current_ang)) * dt
    y = y + v*math.sin(angleDiff(theta)) * dt + current_v * math.sin(angleDiff(current_ang))* dt
    theta = angleDiff(theta + omega * dt)

    gps_message.x = x
    gps_message.y = y
    gps_message.x_vel = robot_vx
    gps_message.y_vel = robot_vy
    gps_message.ang_course = ang_course

    pub_gps.publish(gps_message)

    print("x: ", x)
    print("y: ", y)
    print("xvel: ", robot_vx)
    print("yvel: ", robot_vy)
    print("ang_course: ", ang_course)
    # print(state.v)
    # print(state.theta)
    # print("x, y:", state.x, state.y)
    # print(state.y)
    # self.state_est.lat, self.state_est.lon = utm.to_latlon(self.utm_x, self.utm_y, 11, 'S')

    # keep this to return the updated state



def main():
    rospy.init_node('sim')
    rospy.Subscriber('motor_controller/motor_cmd_reciever', MotorCommand, update_state)
    rospy.spin()


if __name__ == '__main__':
    main()