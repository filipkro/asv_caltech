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

if __name__ == '__main__':
    main()
