import rospy
import rosbag
import math
import numpy as np
import matplotlib.pyplot as plt


print('loading...')
# bag = rosbag.Bag('/media/nvidia/TOSHIBA EXT/Data/Data_2019-07-29-17-26-00.bag')
bag = rosbag.Bag('~/Desktop/Data_2019-08-16-19-23-35.bag')

num_heading = bag.get_message_count('/imu')

ang_imu = np.zeros(num_heading)


def s16(value):
    ''' convert unsigned integer to signed integer'''
    return -(value & 0x8000) | (value & 0x7fff)

def angDiff(value):
    while (value > math.pi):
        value = value - math.pi * 2
    while (value < -math.pi):
        value = value + math.pi * 2
    return value
    
i = 0 # for imu
for topic, msg, t in bag.read_messages(topics=['imu']):
    
    if topic == '/imu':
        ang_heading[i] == msg.data[8]
        i=i+1

        

# filter out the bad data
# s_surface[s_surface >= 32768] = np.nan

plt.figure(1)
plt.plot(np.arange(0,num_heading, ang_heading))
# plt.plot(np.arange(0,num_msgs), s_surface,'b-o',label='surface speed')
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

plt.show()

bag.close()



