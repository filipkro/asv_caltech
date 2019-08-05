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
from sklearn import linear_model


bag = rosbag.Bag('/home/filip/Documents/SURF/asv_ws/src/bagfiles/Test/Test_2019-08-01-10-51-19.bag')
# bag = rosbag.Bag('/home/filip/Documents/SURF/asv_ws/src/bagfiles/Test/2019-08-02-09-27-41.bag')
rospy.init_node('data_processing_node')
listener = tf.TransformListener()
got_transform = False

# default is 1024 point per revolution
angle_increment = math.pi/360.0
x = np.zeros([])
y = np.zeros([])

for topic, msg, t in bag.read_messages(topics=['/os1/scan']):
# for topic, msg, t in bag.read_messages(topics=['/scan']):

    if topic == '/os1/scan':
    # if topic == '/scan':
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
        # x = x[np.logical_and(x!=np.inf, x!=-np.inf)]
        # y = y[np.logical_and(y!=np.inf, y!=-np.inf)]
        x = x[np.logical_and(x!=151.0, x!=-151.0)]
        y = y[np.logical_and(y!=151.0, y!=-151.0)]
        order= np.argsort(x) # sort it, scipy requires strictly ascending order

        # https://stackoverflow.com/questions/45179024/scipy-bspline-fitting-in-python
        # t, c, k= interpolate.splrep(x[order], y[order], s=100, k=1)
        # spline = interpolate.BSpline(t, c, k, extrapolate=False)

        # Robustly fit linear model with RANSAC algorithm
        ransac = linear_model.RANSACRegressor()
        x.shape = (x.size, 1)
        y.shape = (y.size, 1)
        ransac.fit(x, y)

        #Huber
        huber = linear_model.HuberRegressor()
        huber.fit(x, y)

        print('RANSAC', ransac.estimator_.coef_)
        print('RANSAC', ransac.estimator_.intercept_)
        print('Huber', huber.coef_)


        N = 100
        xmin, xmax = x.min(), x.max()
        xx = np.linspace(xmin, xmax, N)
        xx.shape = (xx.size, 1)
        y_ransac = ransac.predict(xx)
        y_huber = huber.predict(xx)

        plt.clf()
        plt.axis([-100, 300, -100, 100])
        plt.plot(x, y, 'bo', label='Original points')
        plt.plot(xx, y_ransac, 'r', label='RANSAC')
        plt.plot(xx, y_huber, 'g', label='Huber')
        plt.pause(0.05)

plt.show()

bag.close()
