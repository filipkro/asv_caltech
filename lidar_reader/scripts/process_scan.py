import rosbag
import tf
import rospy
import numpy as np
import time
import ros_numpy
import matplotlib.pyplot as plt
import math
import scipy.interpolate as interpolate
from sensor_msgs import point_cloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


bag = rosbag.Bag('/media/nvidia/TOSHIBA EXT/Test_2019-08-01-10-51-19.bag')
rospy.init_node('data_processing_node')
listener = tf.TransformListener()
got_transform = False

# default is 1024 point per revolution
angle_increment = math.pi/360.0
x = np.zeros([])
y = np.zeros([])

for topic, msg, t in bag.read_messages(topics=['/os1/scan']):

    
    if topic == '/os1/scan':
        print('we here')
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_inc = msg.angle_increment
        num_ranges = len(msg.ranges)
        x = np.zeros(num_ranges)
        y = np.zeros(num_ranges)

        for i in range(len(msg.ranges)):
            angle = angle_min + i * angle_inc
            x[i] = msg.ranges[i] * math.cos(angle)
            y[i] = msg.ranges[i] * math.sin(angle)

        # remove inf values
        x = x[np.logical_and(x!=np.inf, x!=-np.inf)]
        y = y[np.logical_and(y!=np.inf, y!=-np.inf)]
        order= np.argsort(x) # sort it, scipy requires strictly ascending order

        # https://stackoverflow.com/questions/45179024/scipy-bspline-fitting-in-python
        t, c, k= interpolate.splrep(x[order], y[order], s=100, k=1)
        spline = interpolate.BSpline(t, c, k, extrapolate=False)


        N = 100
        xmin, xmax = x.min(), x.max()
        xx = np.linspace(xmin, xmax, N)
        
        plt.clf()
        plt.axis([-100, 100, -100, 100])
        plt.plot(x, y, 'bo', label='Original points')
        plt.plot(xx, spline(xx), 'r', label='BSpline')
        plt.pause(0.05)

plt.show()

bag.close()
