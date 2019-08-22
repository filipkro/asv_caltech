import pygmaps
import numpy as np
import itertools

gps = np.load('/home/filip/Documents/SURF/asv_ws/src/bagfiles/kern/trans/gps.npy')

latlon = gps[:,4:6]
trans = np.transpose(latlon)
print(latlon)
print('trans', trans)


np.savetxt("/home/filip/Documents/SURF/asv_ws/src/bagfiles/kern/trans/coords.csv", latlon, delimiter=",",header="lat,lon")
# np.savetxt("/home/filip/Documents/SURF/asv_ws/src/bagfiles/kern/trans/coords.csv", np.dstack((np.arange(1, arr.size+1),arr))[0],"%d,%d",header="Id,Values")


# numpy.savetxt("foo.csv", a, delimiter=",")


########## CONSTRUCTOR: pygmaps.maps(latitude, longitude, zoom) ##############################
# DESC:initialize a map with latitude and longitude of center point
#and map zoom level "15"
# PARAMETER1:latitude (float) latittude of map center point
# PARAMETER2:longitude (float) latittude of map center point
# PARAMETER3:zoom (int) map zoom level 0~20
# RETURN:the instant of pygmaps
#========================================================================================
# gps = np.load('/home/filip/Documents/SURF/asv_ws/src/bagfiles/kern/trans/gps.npy')
# lat_c = gps[len(gps)/2,4]
# lon_c = gps[len(gps)/2,5]
# mymap = pygmaps.gmap(lat_c,lon_c, 20)
#
#
# ########## FUNCTION: setgrids(start-Lat, end-Lat, Lat-interval, start-Lng, end-Lng, Lng-interval) ######
# # DESC:set grids on map
# # PARAMETER1:start-Lat (float), start (minimum) latittude of the grids
# # PARAMETER2:end-Lat (float), end (maximum) latittude of the grids
# # PARAMETER3:Lat-interval (float) grid size in latitude
# # PARAMETER4:start-Lng (float), start (minimum) longitude of the grids
# # PARAMETER5:end-Lng (float), end (maximum) longitude of the grids
# # PARAMETER6:Lng-interval (float) grid size in longitude
# # RETURN:no returns
# #========================================================================================
# # mymap.setgrids(17.45, 17.46, 0.001, 78.29,78.30, 0.001)
#
# ########## FUNCTION: addpoint(latitude, longitude, [color])#############################
# # DESC:add a point into a map and dispaly it, color is optional default is red
# # PARAMETER1:latitude (float) latitude of the point
# # PARAMETER2:longitude (float) longitude of the point
# # PARAMETER3:color (string) color of the point showed in map, using HTML color code
# #HTML COLOR CODE: http://www.computerhope.com/htmcolor.htm
# #e.g. red "#FF0000", Blue "#0000FF", Green "#00FF00"
# # RETURN:no return
# #========================================================================================
# for i in range(len(gps[:,4])):
#     mymap.add_point(gps[i,4], gps[i,5], "#FF0000","Hello")
#
#
# ########## FUNCTION: addradpoint(latitude, longitude, radius, [color])##################
# # DESC: add a point with a radius (Meter) - Draw cycle
# # PARAMETER1:latitude (float) latitude of the point
# # PARAMETER2:longitude (float) longitude of the point
# # PARAMETER3:radius (float), radius in meter
# # PARAMETER4:color (string) color of the point showed in map, using HTML color code
# #HTML COLOR CODE: http://www.computerhope.com/htmcolor.htm
# #e.g. red "#FF0000", Blue "#0000FF", Green "#00FF00"
# # RETURN:no return
# #========================================================================================
# # mymap.addradpoint(17.45,78.29, 150, "#0000FF")
#
#
# ########## FUNCTION: addpath(path,[color])##############################################
# # DESC:add a path into map, the data struceture of Path is a list of points
# # PARAMETER1:path (list of coordinates) e.g. [(lat1,lng1),(lat2,lng2),...]
# # PARAMETER2:color (string) color of the point showed in map, using HTML color code
# #HTML COLOR CODE: http://www.computerhope.com/htmcolor.htm
# #e.g. red "#FF0000", Blue "#0000FF", Green "#00FF00"
# # RETURN:no return
# #========================================================================================
# # path = [(gps[0,4],gps[0,5])]
# # for lat,lon in itertools.izip(gps[:,4],gps[:,5]):
# #     path = [path[:], (lat, lon)]
#
#
# # for i in range(len(gps[:,4])):
# #     path = [path[:], (gps[i,4], gps[i,5])]
# #     print(gps[i,4])
# #     # print(path[:])
# #
# # # path = gps[:,4:5]
# # mymap.add_path(path,"#00FF00")
#
# ########## FUNCTION: draw(file)######################################################
# # DESC:create the html map file (.html)
# # PARAMETER1:file (string) the map path and file
# # RETURN:no return, generate html file in specified directory
# #========================================================================================
# mymap.draw('/home/filip/Documents/SURF/asv_ws/src/bagfiles/kern/trans/mymap.html')
#
# print('OK')
