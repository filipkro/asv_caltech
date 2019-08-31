import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import numpy as np
import rospy
import rosbag
from process_raw_adcp import * # IMPORTANT, CHECK THIS OUT

# Adjust those values and apply it to your processing
BEAM_ANGLE = 20 #degrees
TRANSDUCER_OFFSET = 0.1 #m
COMPASS_OFFSET = 0.0 # put in and apply compass offset here if you wish

# path to your data file. as an example, I put a file in the same directory
# however, it doesnt have to be here. 
filename = 'Data_example.bag' 

########################################
print('Loading bag file...')
bag = rosbag.Bag(filename)

#########################################
## topics that you should know 
# /sensors/imu     imu data
# /adcp/raw        raw adcp data
# /GPS/fix         gps_data
# /os1_cloud_node/points LIDAR point clouds
# /os1_cloud_node/imu  Raw lidar imu
# /os1/imu         filtered imu

def main():
    
    # use this to get number of recorded messages
    num_gps_msg = bag.get_message_count('/GPS/fix') 
    num_adcp_msg = bag.get_message_count('/adcp/raw')
    

    # if you want to access a topic, put the name of that topic here behind topics
    for topic, msg, t in bag.read_messages(topics=['/adcp/raw', '/GPS/fix', '/sensors/imu']):
        
        # Big picture: 
        # The system recorded everything coming to it (IMU, GPS, ADCP) into one big file
        # in chronological order as they come in. Here, we're choosing the ones we're interested in
        # with the "topics" option above and accessing them in chronological order.
        # Unfortunately I do not have time to make it into a .csv or other more 
        # accessible format. But you can totally do that on your own with 
        # the data you're interested in. For adcp data, go to the "process_raw_adcp.py"
        # and see what kind of data we're extracting, and then set the data 
        # you want as the return to the function "extract_data". You can get the 
        # return here and then store it into a csv file that you want.  

        if topic == '/adcp/raw':
            # for specific data you want to extrat, see process_raw_adcp.py
            # you can add more data to your function return if you like
            adcp_data = extract_data(msg.data)
            print(adcp_data)
        elif topic == '/GPS/fix':
            # use msg.latitude, msg.longitude to access lat and lng
            # if you want to know what else the gps returned, 
            # print(msg) will show you the available field, 
            # simply access them with msg.(field_of_interest)
            print(msg.latitude, msg.longitude)

        elif topic == "/sensors/imu":
            # this is the output from the other imu onboard
            # an array with: 
            #       [acc_x, acc_y, acc_z, gx, gy, gz, roll, pitch, yaw]
            # note: there is definitely a compass offset. Apply an offset
            # here as you see fit. 
            print(msg.data)

if __name__ == '__main__':
    main()
