import numpy as np
import matplotlib.pyplot as plt
#
#
gps = np.load('/home/filip/Documents/SURF/asv_ws/src/bagfiles/kern/trans/gps.npy')
#
error = np.load('/home/filip/Documents/SURF/asv_ws/src/bagfiles/kern/long_run/errors.npy')

calcs = np.load('/home/filip/Documents/SURF/asv_ws/src/bagfiles/kern/long_run/calcs.npy')

# print(calcs)
#
# plt.figure(1)
# plt.plot(calcs[2,:], calcs[0,:])
# plt.figure(2)
# plt.plot(calcs[2,:], calcs[1,:])
# plt.show()



# plt.figure(1)
# plt.plot(t_imu, ang_imu)
# # plt.plot(np.arange(0,num_msgs), s_surface,'b-o',label='surface speed')
# plt.plot(np.arange(0,num_msgs), s_bt,'r-o',label='bt speed')
# plt.plot(np.arange(0,num_msgs), s_surface - s_bt, 'g-o', label='current speed')
# plt.legend('relative water speed' 'boat speed', 'water speed')

# plt.figure(1)
# plt.plot(gps[:,0], gps[:,1])
# plt.axis('equal')
# plt.show()
#

start = np.where(error[5,:] > 200)
end = len(error[5,:])-1
print(start[0])
print(end)

plt.figure(1)
plt.plot(error[5, 962:end] - error[5,962], error[0,962:end])
plt.xlabel('Time / s')
plt.ylabel('Error / m')
plt.title('Velocity error in waypoint controller')
plt.figure(2)
plt.plot(error[5, 900:end]- error[5,900], error[1,900:end])
plt.xlabel('Time / s')
plt.ylabel('Error / rad')
plt.title('Angle error in waypoint controller')
# plt.plot(error[5,:], error[0,:])
# plt.figure(3)
# plt.plot(error[5,:], error[1,:])
# plt.figure(4)
# plt.plot(error[5,:], error[2,:])
# plt.figure(5)
# plt.plot(error[5,:], error[3,:])
# plt.figure(6)
# plt.plot(error[5,:], error[4,:])

plt.show()

# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.basemap import Basemap
#
# f = plt.figure(figsize(10,7.5))
# m = Basemap(projection="mill", lon_0=0)
#
# m.drawcoastlines()
# m.drawparallels(np.arange(-90,91,30),labels=[1,0,0,0])
# m.drawmeridians(np.arange(-180,181,60), labels=[0,0,0,1])
#
# x,y = m(lon, lat)
# m.plot(x, y, color="red", latlon=False, marker='.', linestyle='None')
