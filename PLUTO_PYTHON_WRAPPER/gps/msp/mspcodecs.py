class MSPCommands:
    MSP_API_VERSION = 1
    MSP_FC_VARIANT = 2
    MSP_FC_VERSION = 3
    MSP_BOARD_INFO = 4
    MSP_BUILD_INFO = 5

    # Cleanflight original features
    MSP_CHANNEL_FORWARDING = 32
    MSP_SET_CHANNEL_FORWARDING = 33
    MSP_MODE_RANGES = 34
    MSP_SET_MODE_RANGE = 35
    MSP_RX_CONFIG = 44
    MSP_SET_RX_CONFIG = 45
    MSP_LED_COLORS = 46
    MSP_SET_LED_COLORS = 47
    MSP_LED_STRIP_CONFIG = 48
    MSP_SET_LED_STRIP_CONFIG = 49
    MSP_ADJUSTMENT_RANGES = 52
    MSP_SET_ADJUSTMENT_RANGE = 53
    MSP_CF_SERIAL_CONFIG = 54
    MSP_SET_CF_SERIAL_CONFIG = 55
    MSP_SONAR = 58
    MSP_PID_CONTROLLER = 59
    MSP_SET_PID_CONTROLLER = 60
    MSP_ARMING_CONFIG = 61
    MSP_SET_ARMING_CONFIG = 62
    MSP_DATAFLASH_SUMMARY = 70
    MSP_DATAFLASH_READ = 71
    MSP_DATAFLASH_ERASE = 72
    MSP_LOOP_TIME = 73
    MSP_SET_LOOP_TIME = 74
    MSP_FAILSAFE_CONFIG = 75
    MSP_SET_FAILSAFE_CONFIG = 76
    MSP_RXFAIL_CONFIG = 77
    MSP_SET_RXFAIL_CONFIG = 78
    MSP_SDCARD_SUMMARY = 79
    MSP_BLACKBOX_CONFIG = 80
    MSP_SET_BLACKBOX_CONFIG = 81
    MSP_TRANSPONDER_CONFIG = 82
    MSP_SET_TRANSPONDER_CONFIG = 83

    MSP_LED_STRIP_MODECOLOR = 127
    MSP_SET_LED_STRIP_MODECOLOR = 221

    # Multiwii MSP commands
    MSP_IDENT = 100
    MSP_STATUS = 101
    MSP_RAW_IMU = 102
    MSP_SERVO = 103
    MSP_MOTOR = 104
    MSP_RC = 105
    MSP_RAW_GPS = 106
    MSP_COMP_GPS = 107
    MSP_ATTITUDE = 108
    MSP_ALTITUDE = 109
    MSP_ANALOG = 110
    MSP_RC_TUNING = 111
    MSP_PID = 112
    MSP_BOX = 113
    MSP_MISC = 114
    MSP_MOTOR_PINS = 115
    MSP_BOXNAMES = 116
    MSP_PIDNAMES = 117
    MSP_WP = 118
    MSP_BOXIDS = 119
    MSP_SERVO_CONFIGURATIONS = 120
    MSP_3D = 124
    MSP_RC_DEADBAND = 125
    MSP_SENSOR_ALIGNMENT = 126

    MSP_SET_RAW_RC = 200
    MSP_SET_RAW_GPS = 201
    MSP_SET_PID = 202
    MSP_SET_BOX = 203
    MSP_SET_RC_TUNING = 204
    MSP_ACC_CALIBRATION = 205
    MSP_MAG_CALIBRATION = 206
    MSP_SET_MISC = 207
    MSP_RESET_CONF = 208
    MSP_SET_WP = 209
    MSP_SELECT_SETTING = 210
    MSP_SET_HEAD = 211
    MSP_SET_SERVO_CONFIGURATION = 212
    MSP_SET_MOTOR = 214
    MSP_SET_3D = 217
    MSP_SET_RC_DEADBAND = 218
    MSP_SET_RESET_CURR_PID = 219
    MSP_SET_SENSOR_ALIGNMENT = 220

    # Additional baseflight commands that are not compatible with MultiWii
    MSP_UID = 160  # Unique device ID
    MSP_ACC_TRIM = 240  # get acc angle trim values
    MSP_SET_ACC_TRIM = 239  # set acc angle trim values
    MSP_GPS_SV_INFO = 164  # get Signal Strength

    # Additional private MSP for baseflight configurator
    MSP_RX_MAP = 64  # get channel map (also returns number of channels total)
    MSP_SET_RX_MAP = 65  # set rc map numchannels to set comes from MSP_RX_MAP
    MSP_BF_CONFIG = 66  # baseflight-specific settings that aren't covered elsewhere
    MSP_SET_BF_CONFIG = 67  # baseflight-specific settings save
    MSP_SET_REBOOT = 68  # reboot settings
    MSP_BF_BUILD_INFO = 69  # build date as well as some space for future expansion

    MSP_SERVO_MIX_RULES = 241
    MSP_SET_SERVO_MIX_RULE = 242

    MSP_EEPROM_WRITE = 250

    MSP_DEBUGMSG = 253
    MSP_DEBUG = 254

    def __init__(self):
        self.state = 0
        self.message_direction = 1
        self.code = 0
        self.message_length_expected = 0
        self.message_length_received = 0
        self.message_buffer = None
        self.message_buffer_uint8_view = None
        self.message_checksum = 0

        self.callbacks = []
        self.packet_error = 0
        self.unsupported = 0

        self.ledDirectionLetters = ['n', 'e', 's', 'w', 'u', 'd']  # in LSB bit order
        self.ledFunctionLetters = ['i', 'w', 'f', 'a', 't', 'r', 'c', 'g', 's', 'b', 'l']  # in LSB bit order
        self.ledBaseFunctionLetters = ['c', 'f', 'a', 'l', 's', 'g', 'r']  # in LSB bit order
        self.ledOverlayLetters = ['t', 'o', 'b', 'n', 'i', 'w']  # in LSB bit order

        self.last_received_timestamp = None
        self.analog_last_received_timestamp = None

        self.supportedBaudRates = [  # 0 based index.
            'AUTO',
            '9600',
            '19200',
            '38400',
            '57600',
            '115200',
            '230400',
            '250000',
        ]

        self.serialPortFunctions = {
            'MSP': 0,
            'GPS': 1,
            'TELEMETRY_FRSKY': 2,
            'TELEMETRY_HOTT': 3,
            'TELEMETRY_MSP': 4,
            'TELEMETRY_LTM': 4,  # LTM replaced MSP
            'TELEMETRY_SMARTPORT': 5,
            'RX_SERIAL': 6,
            'BLACKBOX': 7,
            'TELEMETRY_MAVLINK': 8
        }

    def read(self, read_info):
        data = bytearray(read_info['data'])

        for byte in data:
            if self.state == 0:  # sync char 1
                if byte == 36:  # $
                    self.state += 1
                print('state: $')

            elif self.state == 1:  # sync char 2
                if byte == 77:  # M
                    self.state += 1
                else:  # restart and try again
                    self.state = 0
                print('state: M')

            elif self.state == 2:  # direction (should be >)
                self.unsupported = 0
                if byte == 62:  # >
                    self.message_direction = 1
                elif byte == 60:  # <
                    self.message_direction = 0
                elif byte == 33:  # !
                    self.unsupported = 1

                print('state: >')
                self.state += 1

            elif self.state == 3:
                self.message_length_expected = byte
                self.message_checksum = byte

                # setup arraybuffer
                self.message_buffer = bytearray(self.message_length_expected)
                self.message_buffer_uint8_view = memoryview(self.message_buffer)

                self.state += 1

            elif self.state == 4:
                self.code = byte
                self.message_checksum ^= byte

                if self.message_length_expected > 0:
                    # process payload
                    self.state += 1
                else:
                    # no payload
                    self.state += 2

            elif self.state == 5:  # payload
                self.message_buffer_uint8_view[self.message_length_received] = byte
                self.message_checksum ^= byte
                self.message_length_received += 1

                if self.message_length_received >= self.message_length_expected:
                    self.state += 1

            elif self.state == 6:
                if self.code == 1:
                    mydata = memoryview(self.message_buffer)
                    print('Data: ' + str(mydata[0]) + str(mydata[1]) + str(mydata[2]))
                    print('CheckSum Check: ' + str(self.message_checksum) + ' ' + str(byte))

                if self.message_checksum == byte:
                    # message received, process
                    self.process_data(self.code, self.message_buffer, self.message_length_expected)
                else:
                    print('code: ' + str(self.code) + ' - crc failed')
                    self.packet_error += 1

                # Reset variables
                self.message_length_received = 0
                self.state = 0

            else:
                print('Unknown state detected: ' + str(self.state))

        self.last_received_timestamp = datetime.now()

    def process_data(self, code, message_buffer, message_length):
        data = memoryview(message_buffer)  # DataView (allowing us to view arrayBuffer as struct/union)

        if not self.unsupported:
            if code == 100:  # Example for MSP_IDENT
                print('Using deprecated msp command: MSP_IDENT')
                # Process MSP_IDENT data
            elif code == 101:  # Example for MSP_STATUS
                print('MSP_STATUS data processing')
            # Add other cases for different MSP codes here
            else:
                print('Unknown code detected: ' + str(code))
        else:
            print('FC reports unsupported message error: ' + str(code))

        # Trigger callbacks, cleanup/remove callback after trigger
        for i in range(len(self.callbacks) - 1, -1, -1):  # iterating in reverse because we use .splice which modifies array length
            if i < len(self.callbacks):
                if self.callbacks[i]['code'] == code:
                    # save callback reference
                    callback = self.callbacks[i]['callback']

                    # remove timeout
                    self.callbacks[i]['timer'].cancel()

                    # remove object from array
                    del self.callbacks[i]

                    # fire callback
                    if callback:
                        callback({'command': code, 'data': data, 'length': message_length})

    def send_message(self, code, data, callback_sent=None, callback_msp=None):
        buffer_out = None
        buf_view = None

        # always reserve 6 bytes for protocol overhead !
        if data:
            size = len(data) + 6
            checksum = 0

            buffer_out = bytearray(size)
            buf_view = memoryview(buffer_out)

            buf_view[0] = 36  # $
            buf_view[1] = 77  # M
            buf_view[2] = 60  # <
            buf_view[3] = len(data)
            buf_view[4] = code

            checksum = buf_view[3] ^ buf_view[4]

            for i in range(len(data)):
                buf_view[i + 5] = data[i]
                checksum ^= buf_view[i + 5]

            buf_view[5 + len(data)] = checksum
        else:
            buffer_out = bytearray(6)
            buf_view = memoryview(buffer_out)

            buf_view[0] = 36  # $
            buf_view[1] = 77  # M
            buf_view[2] = 60  # <
            buf_view[3] = 0  # data length
            buf_view[4] = code  # code
            buf_view[5] = buf_view[3] ^ buf_view[4]  # checksum

        obj = {'code': code, 'requestBuffer': buffer_out, 'callback': callback_msp, 'timer': None}

        request_exists = False
        for i in range(len(self.callbacks)):
            if i < len(self.callbacks):
                if self.callbacks[i]['code'] == code:
                    request_exists = True
                    break
            else:
                print("Callback index error: " + str(i))

        if not request_exists:
            obj['timer'] = Timer(5.0, self.timeout_callback, [code, buffer_out])
            obj['timer'].start()

        self.callbacks.append(obj)

        if data or not request_exists:
            serial.send(buffer_out, self.serial_send_callback(callback_sent, buffer_out))

        return True

    def serial_send_callback(self, callback_sent, buffer_out):
        def callback(send_info):
            if send_info.bytes_sent == len(buffer_out):
                if callback_sent:
                    callback_sent()

        return callback

    def timeout_callback(self, code, buffer_out):
        print('MSP data request timed-out: ' + str(code))
        serial.send(buffer_out, False)

    def callbacks_cleanup(self):
        print("MSP-IN CALLBACK CLEANUP")

        for i in range(len(self.callbacks)):
            self.callbacks[i]['timer'].cancel()

        self.callbacks = []

    def disconnect_cleanup(self):
        self.state = 0  # reset packet state for "clean" initial entry (this is only required if user hot-disconnects)
        self.packet_error = 0  # reset CRC packet error counter for next session

        self.callbacks_cleanup()
