#!/usr/bin/env python
import serial
import time
import rospy
import math
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import String
import struct
import datetime

def s16(value):
    '''Change unsigned int to signed int

    Args:
        value (uint): any unsigned int
    Returns:
        (int): signed int'''
    return -(value & 0x8000) | (value & 0x7fff)
    
def extract_data(all_data):
    '''Read an ensemble of ADCP data. 
    Publish with the adcp_pub on topic /adcp/pub
    
    Args:
        all_data (bytestring): an bytestring that contains an ensemble
        (one package) of adcp data
    Return: 
        float list: essential adcp measurements
           [heading, roll, pitch, transducer_depth, bt_ranges, bt_velocities, 
           surface_vel]
           bt_ranges, bt_velocities, surface vel is floats long'''
    all_data = all_data.data
    num_types = all_data[5]
    
    # OFFSETS
    # 1. Fix Leader
    # 2. Variable Leader
    # 3. Velocity Profile
    # 6. Bottom Track
    # 9. Vertical Beam Range
    # 10-13. GPS
    offsets = []
    for i in range(int(num_types.encode('hex'),16)):
        offset = all_data[6+2*i:8+2*i]
        offset_int = int(''.join(reversed(offset)).encode('hex'),16)
        offsets.append(offset_int)
    # print('Offsets: ', offsets)
    # print('Data IDs: ', [all_data[x:x+2] for x in offsets])

    # python3 way of extracting hex value int.from_bytes(data, byteorder='little')
    # python2 way of extracting int(''.join(reversed(data)).encode('hex'), 16)
    # reference https://stackoverflow.com/questions/32014475/unable-convert-to-int-from-bytes

    # FIXED LEADER
    fixed_offset = offsets[0]
    num_beams = all_data[fixed_offset+8]
    num_cells = all_data[fixed_offset+9]
    pings_per_ensemble = int(''.join(reversed(all_data[fixed_offset+10: fixed_offset+12])).encode('hex'),16)
    depth_cell_length = int(''.join(reversed(all_data[fixed_offset+12: fixed_offset+14])).encode('hex'),16)
    coord_transform = all_data[fixed_offset+25]
    heading_alignment = int(''.join(reversed(all_data[fixed_offset+26: fixed_offset+28])).encode('hex'),16)
    # print('Heading alignment: ', heading_alignment)

    # VARIABLE LEADER
    variable_offset = offsets[1]

    transducer_depth = int(''.join(reversed(all_data[variable_offset+14:variable_offset+16])).encode('hex'),16) #1 dm
    heading = s16(int(''.join(reversed(all_data[variable_offset+18:variable_offset+20])).encode('hex'),16)) # degree
    pitch = s16(int(''.join(reversed(all_data[variable_offset+22:variable_offset+24])).encode('hex'),16)) # degree
    roll = s16(int(''.join(reversed(all_data[variable_offset+20:variable_offset+22])).encode('hex'),16)) # degree
    salinity = int(''.join(reversed(all_data[variable_offset+24:variable_offset+26])).encode('hex'),16) # 0 to 40 ppm
    temperature = int(''.join(reversed(all_data[variable_offset+26:variable_offset+28])).encode('hex'),16) # degree
    # print('HEADING', heading)
    # VELOCITY PROFILE
    velocity_profile_offset = offsets[2]
    relative_velocities = []

    # TODO: figure out coordinate transform once more
    for i in range(int(num_cells.encode('hex'),16)):
        start_offset = velocity_profile_offset + 2 + 8*i
        # Average over beams
        vel = []
        for j in range(int(num_beams.encode('hex'),16)):
            curVel = s16(int(''.join(reversed(all_data[start_offset + 2*j : start_offset + 2 + 2*j])).encode('hex'),16))
            # curVel = int.from_bytes(all_data[start_offset + 2*j: start_offset + 2 + 2*j], byteorder='little', signed=True)
            vel.append(curVel)
        relative_velocities.append(vel)

    # BOTTOM TRACK (abbr. bt) (see page 154)

        # Coordinate system for velocity:
            # 1. Earth Axis (default): East, North, Up (right hand orthogonal) 
                # need to set heading alignment (EA), heading bias (EB) correctly
                # make sure heading sensors are active (EZ)
            # 2. Radial Beam Coordinates: "raw beam measurements" (not orthogonal)
            # 3. Instrument coordinates: X, Y, UP, X is directon of beam 2, Y is beam 3. Compass
                # measures the offset of Y from magnetic north
            # 4. Ship coordinates: starboard, forward, mast (pitch, roll, yaw)

    bt_offset = offsets[5]
    bt_pings_per_ensemble = int(''.join(reversed(all_data[bt_offset+2:bt_offset+4])).encode('hex'),16)
    bt_ranges = [] # ranges measurement for each beam (bt = 0 is bad data) # cm
    bt_velocities = [] # there are one more velocity data.. though not sure what it's for?
    beam_percent_good = []
    max_tracking_depth = int(''.join(reversed(all_data[bt_offset+70:bt_offset+72])).encode('hex'),16)

    for i in range(4):
        bt_ranges.append(int(''.join(reversed(all_data[bt_offset+16+i*2:bt_offset+18+i*2])).encode('hex'),16))
        bt_velocities.append(s16(int(''.join(reversed(all_data[bt_offset+24+i*2:bt_offset+26+i*2])).encode('hex'),16)))
        beam_percent_good.append(all_data[bt_offset+40+i])
    
    # VERTICAL BEAM RANGE
    vb_offset = offsets[8]
    vb_range = int(''.join(reversed(all_data[vb_offset+4:vb_offset+8])).encode('hex'),16) # in millimeter

    # GPS Data
    GPS_offsets = offsets[9:13]
    msg_types = []
    msg_sizes = []
    delta_times_bytes = []
    delta_times_double = [] # difference between GPS message and ensemble
    GPS_msg = []

    for g_offset in GPS_offsets:
        msg_size = int(''.join(reversed(all_data[g_offset+4:g_offset+6])).encode('hex'),16)
        msg_types.append(int(''.join(reversed(all_data[g_offset+2:g_offset+4])).encode('hex'),16))
        msg_sizes.append(msg_size)
        delta_times_bytes.append(all_data[g_offset+6:g_offset+14])
        GPS_msg.append(all_data[g_offset+15: g_offset+15+msg_size])

    delta_times_double = [struct.unpack('d', b)[0] for b in delta_times_bytes] # convert to double
    
    surface_vel = relative_velocities[0]
    essential = [heading, roll, pitch, transducer_depth]
    essential = essential + bt_ranges
    essential = essential + bt_velocities
    essential = essential + surface_vel
    # message = Int64MultiArray()
    # message.data = essential
    # adcp_pub.publish(message)
    print(essential)
    
    return essential

def main():
    rospy.init_node('test_extraction')
    rospy.Subscriber("/adcp/raw", String, extract_data)
    rospy.spin()

if __name__ == "__main__":
    main()
