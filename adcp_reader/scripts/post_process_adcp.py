import rospy
import rosbag
import math
import numpy as np
import matplotlib.pyplot as plt


print('loading...')
bag = rosbag.Bag('/media/nvidia/TOSHIBA EXT/Data/Data_2019-07-29-17-26-00.bag')

num_msgs = bag.get_message_count('/adcp/data')
depths = np.zeros(num_msgs) #averaged over four beams
s_surface = np.zeros(num_msgs) # rms value of surface speed
s_bt = np.zeros(num_msgs) # rms value of bottom track
ang_bt = np.zeros(num_msgs) # angle course over ground for the bt velocity
t_adcp = np.zeros(num_msgs)
v_adcp = np.zeros([2,num_msgs])

num_msgs_gps = bag.get_message_count('/GPS/xy_coord')
ang_gps = np.zeros(num_msgs_gps) # angle course over ground for gps
t_gps = np.zeros(num_msgs_gps)
v_gps = np.zeros([2, num_msgs_gps])

def s16(value):
    ''' convert unsigned integer to signed integer'''
    return -(value & 0x8000) | (value & 0x7fff)

def angDiff(value):
    while (value > math.pi):
        value = value - math.pi * 2
    while (value < -math.pi):
        value = value + math.pi * 2
    return value
    
i = 0 # for adcp iteration
j = 0 # for gps
for topic, msg, t in bag.read_messages(topics=['/adcp/data', '/GPS/xy_coord']):
    
    if topic == '/adcp/data':    
        data = msg.data

        # forgot to use unsign int during deployment... fixint it here
        # remember to change it back for correct implementation in the future
        ADCP_yaw = data[0]
        ADCP_roll = data[1]
        ADCP_pitch = data[2]
        ADCP_depths = data[3:7]
        v_bt = [s16(v) for v in data[7:11]] # bottom track velocity (v of boat?)
        v_rel_surface = [s16(v) for v in data[11:15]] # relative surface velocity
        
        # append for plotting
        depths[i] = np.mean(ADCP_depths)
        s_surface[i] = np.sqrt(v_rel_surface[0]**2 + v_rel_surface[1]**2 + \
                            v_rel_surface[2]**2)
        s_bt[i] = np.sqrt(v_bt[0]**2 + v_bt[1]**2 + v_bt[2]**2)
        ang_bt[i] = math.atan2(v_bt[1], v_bt[0])
        v_adcp[:,i] = np.array([v_bt[0], v_bt[1]])

        t_adcp[i] = t.to_sec()     

        i = i + 1

    elif topic == '/GPS/xy_coord':
        ang_gps[j] = msg.ang_course
        v_gps[:,j] = np.array([msg.x_vel, msg.y_vel])
        t_gps[j] = t.to_sec()
        j = j + 1

        

# filter out the bad data
s_surface[s_surface >= 32768] = np.nan

plt.figure(1)
plt.plot(np.arange(0,num_msgs), s_surface,'b-o',label='surface speed')
plt.plot(np.arange(0,num_msgs), s_bt,'r-o',label='bt speed')
plt.plot(np.arange(0,num_msgs), s_surface - s_bt, 'g-o', label='current speed')
plt.legend('relative water speed' 'boat speed', 'water speed')

plt.figure(2)
plt.plot(t_gps, ang_gps)
ang_bt = ang_bt + 135.0/180 * math.pi
ang_bt = [angDiff(angle) for angle in ang_bt]
plt.plot(t_adcp, ang_bt)
plt.legend('gps course angle', 'adcp course angle')

#plt.figure(3)
#plt.plot(t_gps, v_gps[0,:])
#plt.plot(t_adcp, v_adcp[0,:]/100.0)

plt.show()

bag.close()



