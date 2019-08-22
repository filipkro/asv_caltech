import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import numpy as np
import rospy
import rosbag

BEAM_ANGLE = 20 #degrees
TRANSDUCER_OFFSET = 0.1 #m

# Last Bakersfield Rosbag from the TOSHIBA harddrive
smart_transects = '/media/nvidia/TOSHIBA EXT/field_test200819pt5/Data_2019-08-16-15-05-12.bag' # 5MB

########################################
print('Loading bag file...')
data_file = smart_transects
bag = rosbag.Bag(data_file)

# To crop GEOTIFF use:
# gdal_translate -srcwin 3000 9000 4000 3000 input.tif output.tif

# Plot parameters
CELL_RES = 0.4 #cell resolution

# Kalman filter parameters
win = 20 #window size
sigma_slope = 0.1204
sigma_offset = 0.6142

###############################################################################
# Map Helper Functions
###############################################################################

def load_map(filename):
    dataset = gdal.Open(filename)
    data = dataset.ReadAsArray()
    _, height, width = data.shape
    geo_trans = dataset.GetGeoTransform()
    inv_trans = gdal.InvGeoTransform(geo_trans)

    img = mpimg.imread(filename)
    return img, inv_trans, geo_trans, width, height


def read_ADCP_file(data):
    depths = []
    for ensemble in data:
        ensemble_data = ensemble[6:] #Remove '$ADCP,'
        result = read_ensemble(ensemble_data)
        vals = [float(d)*np.cos(BEAM_ANGLE*np.pi/180.) for d in result[6]]
        depths.append(np.average(vals) + TRANSDUCER_OFFSET)
    return -np.asarray(depths)


###############################################################################

#Read data from 1 trial
def main():
    
    num_gps_msg = bag.get_message_count('/GPS/xy_coord')
    x_asv = np.zeros(num_gps_msg)
    y_asv = np.zeros(num_gps_msg)
    g = 0
    min_depth = 0.0
    min_x = 0.0
    min_y = 0.0
    max_x = 0.0
    max_y = 0.0

    # getting min values
    for topic, msg, t in bag.read_messages(topics=['/adcp/data', '/GPS/xy_coord']):
        if topic == '/adcp/data':
            # get current depth
            depths_list = msg.data[4:7]
            cur_depth = -np.mean(depths_list) / 100.0 + TRANSDUCER_OFFSET # (original unit in cm)
            if cur_depth < min_depth:
                min_depth = cur_depth

        elif topic == '/GPS/xy_coord':
            cur_x = msg.x
            cur_y = msg.y
            if min_x > cur_x:
               min_x = cur_x
            
            if min_y > cur_y:
               min_y = cur_y
            if max_x < cur_x:
               max_x = cur_x
            if max_y < cur_y:
               max_y = cur_y

    print('Running Kalman Filter')
    # initialize grid for 1D Kalman Filter
    
    m = int(np.ceil((max_x - min_x)/CELL_RES))
    n = int(np.ceil((max_y - min_y)/CELL_RES))
    baseFloor = min_depth
    B = baseFloor*np.ones((m,n))
    B_var = np.zeros((m,n))
    Bvar = np.zeros((m,n))

    cur_x = 0.0
    cur_y = 0.0
    # read data from the rosbag file
    for topic, msg, t in bag.read_messages(topics=['/adcp/data', '/GPS/xy_coord']):
        if topic == '/adcp/data':
            # get current depth
            depths_list = msg.data[4:7]
            cur_depth = -np.mean(depths_list) / 100.0 + TRANSDUCER_OFFSET # (original unit in cm)

            # get current cell
            i = int(np.floor( (cur_x - min_x) /CELL_RES))
            j = int(np.floor( (cur_y - min_y) /CELL_RES))
            

            for k in range(max(0,i-win), min(m,i+win)):
                for l in range(max(0,j-win), min(n,j+win)):
                    dist2 = 0.1+CELL_RES*((k-i)**2+(l-j)**2)**0.5
                    
                    if B[k][l] == baseFloor:
                        B[k][l] = cur_depth
                        Bvar[k][l] = (dist2*sigma_slope+sigma_offset)**2
                    else:
                        var = (dist2*sigma_slope+sigma_offset)**2
                        cur_K = float(Bvar[k][l])/(Bvar[k][l] + var)
                        B[k][l] = B[k][l]+cur_K*(cur_depth - B[k][l])
                        Bvar[k][l] = Bvar[k][l]-cur_K*Bvar[k][l];

        elif topic == '/GPS/xy_coord':
            cur_x = msg.x
            cur_y = msg.y
            x_asv[g] = cur_x
            y_asv[g] = cur_y
            g+=1
            
    


#    if len(ASV_nor) != len(Z):
#        print('Size mismatch!', len(ASV_nor), len(Z))
#        ASV_nor = ASV_nor[:-1]
#        ASV_eas = ASV_eas[:-1]

#    min_depth = Z.min()

#    #Normalize positions
#    min_x = min(ASV_nor)
#    min_y = min(ASV_eas)
#    X = [v - min_x for v in ASV_nor]
#    Y = [v - min_y for v in ASV_eas]
#    max_x = max(X)
#    max_y = max(Y)

#    # Method 0: 1D Kalman Filter
#    m = int(np.ceil(max_x/CELL_RES))
#    n = int(np.ceil(max_y/CELL_RES))
#    baseFloor = min_depth
#    B = baseFloor*np.ones((m,n))
#    Bvar = np.zeros((m,n))

#    for t in range(len(ASV_nor)):
#        cur_x = X[t]
#        cur_y = Y[t]
#        cur_alt = Z[t]

#        i = int(np.floor(cur_x/CELL_RES))
#        j = int(np.floor(cur_y/CELL_RES))

#        for k in range(max(0,i-win), min(m, i+win)):
#            for l in range(max(0,j-win), min(n,j+win)):
#                dist2 = 0.1+CELL_RES*((k-i)**2+(l-j)**2)**0.5
#                    
#                if B[k][l] == baseFloor:
#                    B[k][l] = cur_alt
#                    Bvar[k][l] = (dist2*sigma_slope+sigma_offset)**2
#                else:
#                    var = (dist2*sigma_slope+sigma_offset)**2
#                    cur_K = float(Bvar[k][l])/(Bvar[k][l] + var)
#                    B[k][l] = B[k][l]+cur_K*(cur_alt - B[k][l])
#                    Bvar[k][l] = Bvar[k][l]-cur_K*Bvar[k][l];

#    ###########################################################################
    Y_plot, X_plot = np.meshgrid(np.arange(min_y, min_y + n*CELL_RES, CELL_RES), np.arange(min_x, min_x + m*CELL_RES, CELL_RES))
#    
#    # 2) Depth surface map
    ax1 = plt.figure(figsize=(8,6)).gca(projection='3d')
    ax1.plot_surface(X_plot, Y_plot, B, cmap=cm.viridis, edgecolor='none') #depths
    ax1.plot(x_asv, y_asv, np.zeros(num_gps_msg), color='red') #ASV path
#    ax1.view_init(200, -50)
    ax1.set_zlabel('Depth (m)')
    #ax1.invert_zaxis()
    ax1.set_xlabel('Easting (m)')
    ax1.set_ylabel('Northing (m)')
    ax1.set_title('River Bathymetry Plot')

#    # 3) Scatter plot of raw data
#    ax2 = plt.figure(figsize=(8,6)).gca(projection='3d')

#    ax2.scatter(ASV_eas, ASV_nor, Z, c=Z, cmap=cm.viridis)
#    ax2.plot(ASV_eas, ASV_nor, np.zeros(len(X)), color='red') #ASV path
#    ax2.view_init(200, -50)
#    ax2.set_zlabel('Depth (m)')
#    ax2.invert_zaxis()
#    ax2.set_xlabel('Easting (m)')
#    ax2.set_ylabel('Northing (m)')



    plt.show()

if __name__ == '__main__':
    main()
