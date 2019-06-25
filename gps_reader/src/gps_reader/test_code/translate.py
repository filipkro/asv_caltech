import numpy as np

def get_coordinates(origLat, origLon, lat, lon):
    EARTH_RADIUS = 6371000;
    x = (lon - origLon) * np.pi/180 * np.cos((lat + origLat)/2 * np.pi/180) * EARTH_RADIUS
    y = (lat - origLat) * np.pi/180 * EARTH_RADIUS

    return x,y
