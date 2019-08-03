import rosbag
import tf
import rospy
import numpy as np
import time
import ros_numpy
from sensor_msgs import point_cloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


bag = rosbag.Bag('/media/nvidia/TOSHIBA EXT/Test_2019-07-31-14-53-42.bag')
rospy.init_node('data_processing_node')
listener = tf.TransformListener()
got_transform = False

rot = None
tran = None

for topic, msg, t in bag.read_messages(topics=['/os1/lidar', '/tf']):

    if topic == '/tf' and msg.transforms[0].header.frame_id == 'map':
        tran = msg.transforms[0]#.transform.translation
        #rot = msg.transforms[0].transform.rotation
        got_transform = True

    
    if got_transform == True and topic == '/os1/lidar':
        print('we here')
        t = time.time()

        cloud_out = do_transform_cloud(msg, tran)
        cloud_p = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(cloud_out)
        cloud_p = cloud_p[np.logical_and(cloud_p[:,2] < 0.2 , cloud_p[0,:] != 0 , cloud_p[1,:] != 0 )]     
        print(cloud_p.shape)

        print(time.time() - t)
        break

bag.close()

