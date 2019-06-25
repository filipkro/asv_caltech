import serial


class GPS_ros:

    def __init__(self):

        self.GPS_PORT = '/dev/serial/by-id/usb-FTDI_USB_Serial_\
                    Converter_FT8VWDEF-if00-port0' #Pi ADCP
        self.GPS_ser = None


    def setup_GPS(self):
        self.GPS_ser = serial.Serial(self.GPS_PORT, bytesize = 8)
        self.GPS_ser.baudrate = 19200
        self.GPS_ser.parity = serial.PARITY_NONE
        self.GPS_ser.stop_bits = serial.STOPBITS_ONE
        self.GPS_ser.write("$JASC,GPGGA,20".encode() + b'\r\n')
        time.sleep(0.1)
        self.GPS_ser.write("$JASC,GPVTG,20".encode() + b'\r\n')
        self.GPS_ser.flushInput()


###############################################################################
# wha's needed, but GPS Functions
###############################################################################
    def update_GPS(self):
        # TODO: if no lock add a hard stop
        while True:
            if self.terminate == True:
                break
            else:
                if self.environment.GPS_ser.in_waiting != 0:
                    data_str = self.environment.GPS_ser.readline()

                    # Data Logging
                    if self.log_data == True:
                        current_state = [self.state_est.x, self.state_est.y, self.state_est.theta, self.state_est.v_course, self.state_est.ang_course, self.rudder, self.port, self.strboard, self.cur_des_point.x, self.cur_des_point.y]
                        current_state_str = "$CTRL," + ",".join(map(str,current_state)) + "###"
                        self.all_data_f.write(current_state_str.encode())

                        self.all_data_f.write("$GPS,".encode() + data_str + b'###')
                    try:
                        self.process_GPS(data_str)
                    except:
                        continue

        # self.gps_f.close()
        self.environment.GPS_ser.close()

def process_GPS(self, data):
    ''' Convert GPS (lat, lon) -> UTM -> Local XY '''
    try:
        data_decoded = data.decode()
    except:
        return
    # data_decoded = '$GPGGA,194502.00,3526.9108198,N,11854.8502196,W,2,15,0.8,142.610,M,-29.620,M,7.0,0131*78'
    raw_msg = data_decoded.split(',')
    if raw_msg[0] == '$GPGGA':
        self.GPS_fix_quality = raw_msg[6]
        if self.GPS_fix_quality == '0' or self.GPS_fix_quality == '\x00' or self.GPS_fix_quality == '\x000':
            self.GPS_received = False
            # print('Bad GPS, no fix :(')
        else:
            if "\x00" in raw_msg[2]:
                return
            if "\x00" in raw_msg[4]:
                return
            self.GPS_raw_msg = raw_msg
            self.GPS_Time = raw_msg[1]
            self.state_est.lat = self.str_to_coord(raw_msg[2])
            self.state_est.lon = -self.str_to_coord(raw_msg[4])
            self.utm_x, self.utm_y, _, _ = utm.from_latlon(self.state_est.lat, self.state_est.lon)
            self.state_est.x = self.utm_x
            self.state_est.y = self.utm_y
            self.GPS_received = True

            if self.first_GPS:
                self.cur_des_point.x = self.utm_x
                self.cur_des_point.y = self.utm_y
                self.first_GPS = False
    elif raw_msg[0] == '$GPVTG':
        # print(raw_msg)
        if len(raw_msg) < 10:
            return
        if raw_msg[9] == 'N' or raw_msg[1] == "":
            # not valid data
            return
        else:
            # print(raw_msg)
            speed_msg = ''
            course_msg = ''
            if "\x00" in raw_msg[7]:
                for i in range(len(raw_msg[7])):
                    if raw_msg[7][i] == "\x00":
                        speed_msg += "0"
                    else:
                        speed_msg += raw_msg[7][i]
            else:
                speed_msg = raw_msg[7]

            if "\x00" in raw_msg[1]:
                for i in range(len(raw_msg[1])):
                    if raw_msg[1][i] == "\x00":
                        course_msg += "0"
                    else:
                        course_msg += raw_msg[1][i]
            else:
                course_msg = raw_msg[1]
            try:
                self.GPS_speed = float(speed_msg)*1000/3600 # from km/hr to m/s
                self.GPS_course = float(course_msg) # course over ground in degrees
            except:
                return
            self.state_est.v_course = self.GPS_speed
            self.state_est.ang_course = self.angleDiff((-self.GPS_course + 360 + 90)/180.0 * math.pi)
            # self.GPS_course = (-self.state_est.ang_course + 90)/180 * math.pi

            # print("Speed %f, Course %f" % (self.state_est.v_course, self.state_est.ang_course))




if __name__ == '__main__':
    main()
