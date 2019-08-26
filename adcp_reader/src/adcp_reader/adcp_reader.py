#!/usr/bin/env python
import serial
import time
import rospy
import math
from std_msgs.msg import Int64MultiArray
import struct
import datetime

"""ADCP Reader

This script includes implementation for the adcp_reader_node 
for the adcp_reader packages. It starts by setting up serial
connection to the serial to usb chip that connects to the 
ADCP and then continuously query and publish to the /adcp/data
topic. Since the ADCP ensemble (data packages in forms of bytestring) 
are too long, we extract only the essential and save the rest 
to a separate .bin file. 

Example:
    rosrun adcp_reader adcp_reader_node.py

Attributes:
    adcp_ser: the adcp serial port from pyserial
    adcp_f: the file to write adcp ensemble data to. 
    adcp_filename: name for the ensemble data file 
    adcp_pub: ros publisher for pubslihing 'adcp/data'

Todo:
    * make adcp_f a rosparam that can be written to

"""

adcp_ser = None
ANGLE_OFFSET = 45
adcp_f = None
adcp_filename = ''
adcp_pub = rospy.Publisher('adcp/data', Int64MultiArray, queue_size=10)

def setup_adcp():
    '''Initialize adcp serial port, sending appropriate 
    messages to configure it correctly
    
    Args:
        None
    Returns:
        None'''
    global adcp_ser, adcp_f, adcp_filename

    # serial to usb chip name for the adcp 
    adcp_port = '/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0'
    adcp_ser = serial.Serial(adcp_port, 115200, stopbits=serial.STOPBITS_ONE)
    adcp_ser.flush()
    print("Starting ADCP communication")
    adcp_ser.write(b'+++')
    time.sleep(0.5)
    s = read_ADCP_response(verbose=True)
    print('Startup message: ', s)

    # setting the coordinate transformation
    # report readings in earth coorindate
    send_ADCP(b'EX11110')
    s = read_ADCP_response(verbose=True)
    time.sleep(0.1)
    print("Coordinate message: ", s)

    # setting the adcp compass offset
    send_ADCP(b'EA+04500')
    s = read_ADCP_response(verbose=True)
    time.sleep(0.1)
    print("Angle offset message: ", s)

    # log adcp data
    time_stamp = datetime.datetime.now().replace(microsecond=0).strftime('%y-%m-%d %H.%M.%S')
    adcp_filename = '/media/nvidia/37A33B8748E7DD2A/Data/ADCP' + time_stamp + ".bin"
    adcp_f = open(adcp_filename, 'wb')

def send_ADCP(command):
    '''send a command to adcp and return the response
    
    Args:
        command (bytestring): command to send to adcp. See official adcp doc
    Return:
        bytestring: response from the adcp
        (double check on this)'''
    global adcp_ser
    adcp_ser.write(command + b'\r\n')
    s = read_ADCP_response(verbose=True)
    return s

def read_ADCP_response(verbose=False):
    '''obstain a response from the adcp
    Args:
        verbose (bool): optional argument to print all response
    Returns:
        bytestring: response from the adcp'''
    global adcp_ser
    response = b''
    cur_line = b''
    while True:
        s = adcp_ser.read()
        response += s
        cur_line += s
        if verbose and s == b'\n':
            print(cur_line)
            cur_line = b''

        if b'>' in response: #stop character
            return response
    return

def stop_ping():
    """ Send command to ADCP to stop sending data packages"""
    global adcp_ser
    print('Stopping Pings')
    adcp_ser.write(b'CSTOP\r\n') 

def start_ping():
    """ Send command to ADCP to start sending data packages"""
    global adcp_ser
    print("Requesting Pings")
    adcp_ser.write(b'CS\r\n')
    adcp_ser.read(10) #response includes command 

#############################################
######### Reading Data ######################
#############################################

def read_ensemble(verbose=False):
    '''Parse the input data, check for checksum, if it is valid
    extract the part that contains the ensemble. Write data 
    to adcp_f
    
    Args:
        verbose (bool): optional argument to whether to print out 
            everything read
    Returns:
        all_data (byte_string): an ensemble of data from the adcp
        '''

    global adcp_ser, adcp_f, adcp_pub
    header = adcp_ser.read(2)
    if header != b'\x7f\x7f':
        print('ERROR no header: ', header)
    
    num_bytes = adcp_ser.read(2)
    # bytes_to_checksum = int.from_bytes(num_bytes, byteorder='little')-4

    bytes_to_checksum = int(''.join(reversed(num_bytes)).encode('hex'),16) - 4
    if verbose:
        print('Num: ', bytes_to_checksum)
    
    data = adcp_ser.read(bytes_to_checksum)
    if verbose:
        print('Data: ', data)

    #use checksum to verify no errors
    checksum = adcp_ser.read(2)
    checksum_int = int(''.join(reversed(checksum)).encode('hex'),16)
    datasum_int = sum(bytearray(b'\x7f\x7f' + num_bytes + data)) % 2**16

    if checksum_int != datasum_int:
        print('ERROR: ', checksum_int, datasum_int)

    #read data to file
    all_data = b'\x7f\x7f' + num_bytes + data + checksum
    adcp_f.write(all_data)

    return all_data

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
    message = Int64MultiArray()
    message.data = essential
    adcp_pub.publish(message)
    
    return essential

def angleDiff(angle):
    '''Bound an angle between -pi and +pi in radians
    
    Args:
        angle (float): angle to be bounded
    Returns:
        float: angle that is bounded
    '''
    while angle > math.pi:
        angle = angle - 2 * math.pi
    while angle < -math.pi:
        angle = angle + 2 * math.pi

    return angle

def angleDiff_deg(angle):
    '''Bound an angle between -pi and +pi in degrees
    
    Args:
        angle (float): angle to be bounded
    Returns:
        float: angle that is bounded in degrees
    '''
    while angle > 180:
        angle = angle - 2 * 180
    while angle < -180:
        angle = angle + 2 * 180

    return angle

# make this the main()
def main():
    '''Setup serial port, start ping and then continously read and extract data 
    from the adcp'''

    # setup adcp, start ping and init node
    setup_adcp()
    start_ping()
    rospy.init_node('adcp_reader')

    while not rospy.is_shutdown():
        ensemble = read_ensemble(verbose=False) # read an ensemble of data 
        data = extract_data(ensemble) # extract data from the ensemble 
        ADCP_heading = angleDiff_deg(data[0] - ANGLE_OFFSET)
        ADCP_roll = data[1]
        ADCP_pitch = data[2]
        depths = data[3:7]
        bt_boat_beams = data[7:11]
        relative_surface_velocities = data[11:15]
        # ADCP_received = True

    stop_ping()
    adcp_ser.close()
    adcp_f.close()
        # self.adcp_f.close()
        # if self.log_data == True:
        #     self.all_data_f.close()
        
