import rospy
import rosbag
import math
import numpy as np
import matplotlib.pyplot as plt


print('loading...')
# bag = rosbag.Bag('/media/nvidia/TOSHIBA EXT/Data/Data_2019-07-29-17-26-00.bag')
# bag = rosbag.Bag('/home/filip/Documents/SURF/asv_ws/src/bagfiles/kern/long_run/long_run2_repaired')
bag = rosbag.Bag('/home/filip/Documents/SURF/asv_ws/src/bagfiles/kern/long_run/long_run2_repaired.bag')



num_heading = bag.get_message_count('/sensors/imu')
num_motor_cmds = bag.get_message_count('/motor_controller/motor_cmd_reciever')
num_ref = bag.get_message_count('/ref/point')
num_gps = bag.get_message_count('/GPS/xy_coord')
num_transect = bag.get_message_count('/transect')
num_adcp = bag.get_message_count('/adcp/data')
num_calcs = bag.get_message_count('/smart/calcs')

ang_imu = np.zeros(num_heading)
t_imu = np.zeros(num_heading)

def s16(value):
    ''' convert unsigned integer to signed integer'''
    return -(value & 0x8000) | (value & 0x7fff)

def angleDiff(value):
    while (value > math.pi):
        value = value - math.pi * 2
    while (value < -math.pi):
        value = value + math.pi * 2
    return value


# cals_msg.data = [d_left, d_right, dist_mid, theta_p, theta_dest, self.xref, self.yref]

# print('3/6')
d_left = np.zeros(num_calcs)
d_right = np.zeros(num_calcs)
t_calcs = np.zeros(num_calcs)
# d_left = np.zeros(num_calcs)
# d_left = np.zeros(num_calcs)
# ypos = np.zeros(num_gps)
# xvel = np.zeros(num_gps)
# yvel = np.zeros(num_gps)
# gps_ang = np.zeros(num_gps)
# lat = np.zeros(num_gps)
# lon = np.zeros(num_gps)
# lat_origin = np.zeros(num_gps)
# lon_origin = np.zeros(num_gps)
# t_gps = np.zeros(num_gps)
i = 0
for topic, msg, t in bag.read_messages(topics=['/smart/calcs']):
    if topic == '/smart/calcs':
        print('/smart/calcs', i, num_gps)
        d_left[i] = msg.data[0]
        d_right[i] = msg.data[1]
        # xvel[i] = msg.x_vel
        # yvel[i] = msg.y_vel
        # gps_ang[i] = msg.ang_course
        # lat[i] = msg.latitude
        # lon[i] = msg.longitude
        # lat_origin[i] = msg.origin_lat
        # lon_origin[i] = msg.origin_lon

        t_calcs[i] = t.to_sec()
        i += 1

np.save('/home/filip/Documents/SURF/asv_ws/src/bagfiles/kern/long_run/calcs', np.column_stack((d_left, d_right, t_calcs)))
print('finishd')

# i = 0 # for imu
# for topic, msg, t in bag.read_messages(topics=['/sensors/imu']):
#
#     if topic == '/sensors/imu':
#         print('imu', i, num_heading)
#         ang_imu[i] = msg.data[8]
#         t_imu[i] = t.to_sec()
#         i=i+1
#
# print(ang_imu)
#
#
# np.save('/home/filip/Documents/SURF/asv_ws/src/bagfiles/kern/trans/imu', np.column_stack((ang_imu, t_imu)))
#
# print('1/6')
# thrust_cmds = np.zeros(num_motor_cmds)
# servo_cmds = np.zeros(num_motor_cmds)
# t_motor_cmds = np.zeros(num_motor_cmds)
# print(num_motor_cmds)
# i = 0
# for topic, msg, t in bag.read_messages(topics=['/motor_controller/motor_cmd_reciever']):
#     if topic == '/motor_controller/motor_cmd_reciever':
#         print('cmds', i, num_motor_cmds)
#         thrust_cmds[i] = msg.port
#         servo_cmds[i] = msg.servo
#         t_motor_cmds[i] = t.to_sec()
#         i += 1
#
# np.save('/home/filip/Documents/SURF/asv_ws/src/bagfiles/kern/trans/cmds', np.column_stack((servo_cmds, thrust_cmds, t_motor_cmds)))
#
# print('2/6')
# xref = np.zeros(num_ref)
# yref = np.zeros(num_ref)
# t_ref = np.zeros(num_ref)
# print(num_ref)
# i = 0
# for topic, msg, t in bag.read_messages(topics=['/ref/point']):
#     if topic == '/ref/point':
#         print('ref', i, num_ref)
#         xref[i] = msg.point.x
#         yref[i] = msg.point.y
#         t_ref[i] = t.to_sec()
#         i += 1
#
# np.save('/home/filip/Documents/SURF/asv_ws/src/bagfiles/kern/trans/ref', np.column_stack((xref, yref, t_ref)))
#
# print('3/6')
# xpos = np.zeros(num_gps)
# ypos = np.zeros(num_gps)
# xvel = np.zeros(num_gps)
# yvel = np.zeros(num_gps)
# gps_ang = np.zeros(num_gps)
# lat = np.zeros(num_gps)
# lon = np.zeros(num_gps)
# lat_origin = np.zeros(num_gps)
# lon_origin = np.zeros(num_gps)
# t_gps = np.zeros(num_gps)
# i = 0
# for topic, msg, t in bag.read_messages(topics=['/GPS/xy_coord']):
#     if topic == '/GPS/xy_coord':
#         print('gps', i, num_gps)
#         xpos[i] = msg.x
#         ypos[i] = msg.y
#         xvel[i] = msg.x_vel
#         yvel[i] = msg.y_vel
#         gps_ang[i] = msg.ang_course
#         lat[i] = msg.latitude
#         lon[i] = msg.longitude
#         lat_origin[i] = msg.origin_lat
#         lon_origin[i] = msg.origin_lon
#
#         t_gps[i] = t.to_sec()
#         i += 1
#
# np.save('/home/filip/Documents/SURF/asv_ws/src/bagfiles/kern/trans/gps', np.column_stack((xpos, ypos, xvel, gps_ang, lat, lon, lat_origin, lon_origin, t_gps)))
#
# print('4/6')
# current_ang = np.zeros(num_adcp)
# current_vel = np.zeros(num_adcp)
# t_adcp = np.zeros(num_adcp)
# i = 0
# for topic, msg, t in bag.read_messages(topics=['/adcp/data']):
#     if topic == '/adcp/data':
#         print('adcp', i, num_adcp)
#         v_bt = np.array([s16(v) for v in msg.data[7:11]]) # bottom track velocity (v of boat?)
#         v_rel_surface = np.array([s16(v) for v in msg.data[11:15]]) # relative surface velocity
#         v_surface = v_rel_surface - v_bt
#         v_angle = math.atan2(v_surface[1], v_surface[0])
#
#         adcp_offset = rospy.get_param('/ADCP/angleOff', 300.0) / 180.0 * math.pi +math.pi
#         v_angle = angleDiff(v_angle+adcp_offset)
#         current_vel[i] = math.sqrt(v_surface[0]**2 + v_surface[1]**2) / 1000.0
#         current_ang[i] = v_angle
#
#         t_ref[i] = t.to_sec()
#         i += 1
#
# np.save('/home/filip/Documents/SURF/asv_ws/src/bagfiles/kern/trans/adcp', np.column_stack((current_vel, current_ang, t_adcp)))
#
# print('5/6')
# p1 = np.zeros(num_transect)
# p2 = np.zeros(num_transect)
# t_gps = np.zeros(num_transect)
# i = 0
# for topic, msg, t in bag.read_messages(topics=['/transect']):
#     if topic == '/transect':
#         print('transect', i, num_transect)
#         xpos[i] = msg.markers.points[0]
#         ypos[i] = msg.markers.points[1]
#
#         t_ref[i] = t.to_sec()
#         i += 1
#
# print('6/6')




# filter out the bad data
# s_surface[s_surface >= 32768] = np.nan
# print(ang_imu.size)
# plt.figure(1)
# plt.plot(t_imu, ang_imu)
# # plt.plot(np.arange(0,num_msgs), s_surface,'b-o',label='surface speed')
# plt.plot(np.arange(0,num_msgs), s_bt,'r-o',label='bt speed')
# plt.plot(np.arange(0,num_msgs), s_surface - s_bt, 'g-o', label='current speed')
# plt.legend('relative water speed' 'boat speed', 'water speed')

# plt.figure(2)
# plt.plot(t_gps, ang_gps)
# ang_bt = ang_bt + 135.0/180 * math.pi
# ang_bt = [angDiff(angle) for angle in ang_bt]
# plt.plot(t_adcp, ang_bt)
# plt.legend('gps course angle', 'adcp course angle')

#plt.figure(3)
#plt.plot(t_gps, v_gps[0,:])
#plt.plot(t_adcp, v_adcp[0,:]/100.0)

# plt.show()

bag.close()
