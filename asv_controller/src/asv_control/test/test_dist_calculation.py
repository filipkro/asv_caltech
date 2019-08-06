import rospy
import rosbag
import math
import numpy as np
import matplotlib.pyplot as plt


print('loading...')
bag = rosbag.Bag('/media/nvidia/TOSHIBA EXT/2019-08-05-16-08-52.bag')

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
cur_heading = 0.0
ang_off = 135.0/180 * math.pi
lidar_off = math.pi

def s16(value):
    ''' convert unsigned integer to signed integer'''
    return -(value & 0x8000) | (value & 0x7fff)

def angDiff(value):
    while (value > math.pi):
        value = value - math.pi * 2
    while (value < -math.pi):
        value = value + math.pi * 2
    return value

#Calculates two transect points (on land) creating a line perpendicular to current
def calculate_transect(theta_c):
    '''Input:
         theta_c: current angle (float) in global frame
       Output:
         [state, state]: two points on the line normal to theta_c '''
    point1 = GPS_data()
    point2 = GPS_data()
    # sample cerain number of points from the sides of the current angle
    distLs, angL, distL = get_distance(theta_c + math.pi/2, 21)
    distRs, angR, distR = get_distance(theta_c - math.pi/2, 21)

    # generate points using simple trig
    point1.x = state_asv[0] + (distR + 10) * math.cos(theta_c - math.pi/2)
    point1.y = state_asv[1] + (distR + 10) * math.sin(theta_c - math.pi/2)
    point2.x = state_asv[0] + (distL + 10) * math.cos(theta_c + math.pi/2)
    point2.y = state_asv[1] + (distL + 10) * math.sin(theta_c + math.pi/2)

    #how should the points be saved for transect controller??
    return [point1, point2]

def get_distance(ang, lidar_inc, ranges, heading, nbr_of_points=5):
    '''Getting the mean distance to the specified angle
        ang: angle to look for shortest range
        nbr_of_points: number of readings to average over'''
    nbr = int(math.floor(nbr_of_points/2))
    inc = lidar_inc
    
    #search_angle = angDiff(heading - ang + math.pi - lidar_off)
    search_angle = angDiff( ang - heading  + math.pi)
    index = int((search_angle)/inc)
    range_sum = ranges[index]
    
    valid_ranges = []
    valid_angles = []

    for k in range(1,nbr+1):
        range_sum += math.cos(inc*k)*(ranges[index+k] + ranges[index-k])
       valid_ranges.append(ranges[index+k])
       valid_angles.append(ang + inc*k)

       valid_ranges.append(ranges[index-k])
       valid_angles.append(ang-inc*k)

    # return mean of nbr_of_points (uneven) closest points
    #return range_sum/(2*nbr+1)
    return valid_ranges, valid_angles, range_sum
    
i_a = 0 # for adcp iteration
j = 0 # for gps
for topic, msg, t in bag.read_messages(topics=['/adcp/data', '/GPS/xy_coord', '/sensors/imu', '/os1/scan']):
    
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
        depths[i_a] = np.mean(ADCP_depths)
        s_surface[i_a] = np.sqrt(v_rel_surface[0]**2 + v_rel_surface[1]**2 + \
                            v_rel_surface[2]**2)
        s_bt[i_a] = np.sqrt(v_bt[0]**2 + v_bt[1]**2 + v_bt[2]**2)
        ang_bt[i_a] = math.atan2(v_bt[1], v_bt[0])
        v_adcp[:,i_a] = np.array([v_bt[0], v_bt[1]])

        t_adcp[i_a] = t.to_sec()     

        i_a = i_a + 1

    elif topic == '/GPS/xy_coord':
        ang_gps[j] = msg.ang_course
        v_gps[:,j] = np.array([msg.x_vel, msg.y_vel])
        t_gps[j] = t.to_sec()
        j = j + 1

    elif topic == '/os1/scan':
        print('here')
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_inc = msg.angle_increment
        num_ranges = len(msg.ranges)
        x = np.zeros(num_ranges)
        y = np.zeros(num_ranges)

        rot_matrix = np.array([[math.cos(cur_heading), -math.sin(cur_heading)], \
                               [math.sin(cur_heading), math.cos(cur_heading)]])
        
        for i in range(len(msg.ranges)):
            angle = angle_min + i * angle_inc
            xy = np.array([[msg.ranges[i] * math.cos(angle)], [msg.ranges[i] * math.sin(angle)]])
            xy_rot = np.matmul(rot_matrix, xy); 
           
            x[i] = xy_rot[0][0]
            y[i] = xy_rot[1][0]


        # remove inf values
        x = x[np.logical_and(x!=np.inf, x!=-np.inf)]
        x = x[np.logical_not(np.isnan(x))]
        y = y[np.logical_and(y!=np.inf, y!=-np.inf)]
        y = y[np.logical_not(np.isnan(y))]


        ## now find points that are on both sides of the boat angle
        bt_angle = angDiff(ang_bt[i_a-1] + ang_off)
        right_dist, right_angles = get_distance(-math.pi/2, angle_inc, msg.ranges, cur_heading, 21)
        print(right_angles)
        x_lidar = np.zeros(len(right_dist))
        y_lidar = np.zeros(len(right_dist))
        print(right_dist)
        for z in range(len(x_lidar)):
                    
            x_lidar[z] = right_dist[z] * math.cos(right_angles[z])
            y_lidar[z] = right_dist[z] * math.sin(right_angles[z])
        
        
        plt.clf()
        plt.axis([-50, 50, -50, 100])
        plt.plot(x, y, 'bo', label='Original points')
        plt.plot(x_lidar, y_lidar, 'ro')
        plt.pause(0.05)
    
    elif topic == '/sensors/imu':
        cur_heading = msg.data[8] - math.pi

        

plt.show()

bag.close()



