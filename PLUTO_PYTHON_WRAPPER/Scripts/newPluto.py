import socket
from threading import Thread
import time

# Constants
TRIM_MAX = 1000
TRIM_MIN = -1000
MSP_HEADER_IN = "244d3c"

# MSP Command Codes
MSP_FC_VERSION = 3
MSP_RAW_IMU = 102
MSP_RAW_GPS = 106
MSP_COMP_GPS = 107
MSP_RC = 105
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_ANALOG = 110
MSP_SET_RAW_RC = 200
MSP_ACC_CALIBRATION = 205
MSP_MAG_CALIBRATION = 206
MSP_SET_MOTOR = 214
MSP_SET_ACC_TRIM = 239
MSP_ACC_TRIM = 240
MSP_EEPROM_WRITE = 250
MSP_SET_POS = 216
MSP_SET_COMMAND = 217
MSP_SET_1WIRE = 243
MSP_VARIO = 122
RETRY_COUNT = 3

class pluto:
    def __init__(self):
        # Initialize RC values
        self.rcRoll = 1500
        self.rcPitch = 1500
        self.rcThrottle = 1500
        self.rcYaw = 1500
        self.rcAUX1 = 1500
        self.rcAUX2 = 1000
        self.rcAUX3 = 1500
        self.rcAUX4 = 1000
        
        # Initialize command types
        self.command_type = 0
        self.droneRC = [1500, 1500, 1500, 1500, 1500, 1000, 1500, 1000]
        self.NONE_COMMAND = 0
        
        # Initialize PID control variables
        self.prev_values = [0, 0, 0]
        self.last_time = 0.0000
        self.error = [0.0, 0.0, 0.0]
        self.abserror = [0, 0, 0]
        self.errsum = [0, 0, 0]
        self.now = 0.0000
        self.derr = [0, 0, 0]
        self.sample_time = 60
        self.out_roll = 0.000
        self.out_pitch = 0.000
        self.out_throttle = 0.000
        self.Kp = [0.8, 0.4, 380, 50]
        self.Ki = [0.02, 0.01, 0, 0]
        self.Kd = [18, 25, 10, 0]
        self.drone_position = [0, 0, 0]
        self.drone = [0.0, 0.0, 0.0]
        
        # GPS Position Hold variables
        self.target_gps = None
        self.position_hold_active = False
        self.Kp_pos = [0.1, 0.1]
        self.Ki_pos = [0.01, 0.01]
        self.Kd_pos = [0.5, 0.5]
        self.error_sum_pos = [0.0, 0.0]
        self.prev_error_pos = [0.0, 0.0]
        
        # Flight control commands
        self.TAKE_OFF = 1
        self.LAND = 2
        
        # Connection flags
        self.connected = False
        self.cam_connected = False
        self.TCP_IP = '192.168.4.1'
        self.TCP_PORT = 23
        
        # Start write thread
        self.thread = Thread(target=self.write_function)
        self.thread.start()

    # Connection Management
    def start_write_function(self):
        self.thread = Thread(target=self.write_function)
        self.thread.start()
        
    def connect(self):
        if not self.connected:
            self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                self.client.connect((self.TCP_IP, self.TCP_PORT))
                self.connected = True
                print("Connected to server")
                self.start_write_function()
            except Exception as e:
                print("Error connecting to server:", e)

    def disconnect(self):
        if self.connected:
            self.client.close()
            self.connected = False
            print("Disconnected from server")

    def cam(self):
        self.TCP_IP = '192.168.0.1'
        self.TCP_PORT = 9060
        self.camera_mode = True

    # Drone Control Commands
    def arm(self):
        print("Arming")
        self.rcRoll = 1500
        self.rcYaw = 1500
        self.rcPitch = 1500
        self.rcThrottle = 1000
        self.rcAUX4 = 1500

    def box_arm(self):
        print("Box Arming")
        self.rcRoll = 1500
        self.rcYaw = 1500
        self.rcPitch = 1500
        self.rcThrottle = 1800
        self.rcAUX4 = 1500

    def disarm(self):
        print("Disarming")
        self.rcThrottle = 1300
        self.rcAUX4 = 1200

    def forward(self):
        print("Moving Forward")
        self.rcPitch = 1600

    def backward(self):
        print("Moving Backward")
        self.rcPitch = 1300

    def left(self):
        print("Moving Left")
        self.rcRoll = 1200

    def right(self):
        print("Moving Right")
        self.rcRoll = 1600

    def left_yaw(self):
        print("Yawing Left")
        self.rcYaw = 1300

    def right_yaw(self):
        print("Yawing Right")
        self.rcYaw = 1600

    def reset(self):
        self.rcRoll = 1500
        self.rcThrottle = 1500
        self.rcPitch = 1500
        self.rcYaw = 1500
        self.commandType = 0

    def increase_height(self):
        print("Increasing Height")
        self.rcThrottle = 1800

    def decrease_height(self):
        print("Decreasing Height")
        self.rcThrottle = 1300

    def take_off(self):
        self.disarm()
        self.box_arm()
        print("Taking Off")
        self.commandType = 1

    def land(self):
        self.commandType = 2

    def flip(self):
        print("Performing Flip")
        flip_command = [3]  # Assuming 3 is the command for backflip
        flip_packet = self.create_packet_msp(MSP_SET_COMMAND, flip_command)
        self.send_packet(bytes.fromhex(flip_packet))
        self.throttle_speed(500, 1)

    def throttle_speed(self, value, duration=0):
        no_of_loops = 10 * duration
        self.droneRC[2] = self.clamp_rc(self.rcThrottle + value)
        while no_of_loops > 0:
            self.create_packet_msp(MSP_SET_RAW_RC, self.droneRC)
            no_of_loops -= 1
            time.sleep(0.1)
        if duration == 0:
            self.create_packet_msp(MSP_SET_RAW_RC, self.droneRC)

    # PID Control
    def pid(self, setpoint):
        self.drone_position = self.get_drone_pos()
        self.now = int(round(time.time() * 1000))
        self.timechange = self.now - self.last_time
        if self.timechange > self.sample_time:
            if self.last_time != 0:
                self.error[0] = self.drone_position[0] - setpoint[0]
                self.error[1] = self.drone_position[1] - setpoint[1]
                self.error[2] = self.drone_position[2] - setpoint[2]

                self.abserror[0] = self.error[0]
                self.abserror[1] = self.error[1]
                self.abserror[2] = self.error[2]

                self.errsum[0] += self.error[0] * self.timechange
                self.errsum[1] += self.error[1] * self.timechange
                self.errsum[2] += self.error[2] * self.timechange

                self.derr[0] = (self.error[0] - self.prev_values[0]) / self.timechange
                self.derr[1] = (self.error[1] - self.prev_values[1]) / self.timechange
                self.derr[2] = (self.error[2] - self.prev_values[2]) / self.timechange

                print([self.Kp, self.Ki, self.Kd])
                print(self.error)

                self.rcRoll = int(1500 - (self.Kp[0] * self.error[0]) - (self.Kd[0] * self.derr[0]))
                self.rcPitch = int(1500 + (self.Kp[1] * self.error[1]) + (self.Kd[1] * self.derr[1]))
                self.rcThrottle = int(1500 + (self.Kp[2] * self.error[2]) + (self.Kd[2] * self.derr[2]) - (self.errsum[2] * self.Ki[2]))

                if self.rcThrottle > 2000:
                    self.rcThrottle = self.max_values
                if self.rcThrottle < 1000:
                    self.rcThrottle = self.min_values

                if self.rcPitch > 2000:
                    self.rcPitch = self.max_values
                if self.rcPitch < 1000:
                    self.rcPitch = self.min_values

                if self.rcRoll > 2000:
                    self.rcRoll = self.max_values
                if self.rcRoll < 1000:
                    self.rcRoll = self.min_values

                print([self.rcRoll, self.rcPitch, self.rcThrottle])

                self.prev_values[0] = self.error[0]
                self.prev_values[1] = self.error[1]
                self.prev_values[2] = self.error[2]

            self.last_time = self.now

    def altitude_set_pid(self, alt):
        self.Kp[2] = alt.Kp * 0.06
        self.Ki[2] = alt.Ki * 0.0008
        self.Kd[2] = alt.Kd * 0.3

    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 0.06
        self.Ki[1] = pitch.Ki * 0.0008
        self.Kd[1] = pitch.Kd * 0.3

    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 0.06
        self.Ki[0] = roll.Ki * 0.0008
        self.Kd[0] = roll.Kd * 0.3

    # Utility Functions
    def clamp_rc(self, x: int):
        return max(1000, min(2000, x))

    def rc_values(self):
        return [self.rcRoll, self.rcPitch, self.rcThrottle, self.rcYaw,
                self.rcAUX1, self.rcAUX2, self.rcAUX3, self.rcAUX4]

    def motor_speed(self, motor_index, speed):
        if 0 <= motor_index < 4:
            motor_speeds = [1000, 1000, 1000, 1000]
            motor_speeds[motor_index] = speed
            self.send_request_msp_set_motor(motor_speeds)
        else:
            print("Invalid motor index. Must be between 0 and 3.")

    def trim_left_roll(self):
        print("Trimming Left Roll")
        self.rcRoll = max(TRIM_MIN, self.rcRoll + 100)

    # MSP Request and Response Handling
    def send_request_msp(self, data):
        if self.connected:
            self.client.send(bytes.fromhex(data))
        else:
            if not hasattr(self, 'connection_error_logged'):
                print("Error: Not connected to server")
                self.connection_error_logged = True

    def create_packet_msp(self, msp, payload):
        bf = ""
        bf += MSP_HEADER_IN

        checksum = 0
        if msp == MSP_SET_COMMAND:
            pl_size = 1
        else:
            pl_size = len(payload) * 2

        bf += '{:02x}'.format(pl_size & 0xFF)
        checksum ^= pl_size

        bf += '{:02x}'.format(msp & 0xFF)
        checksum ^= msp

        for k in payload:
            if msp == MSP_SET_COMMAND:
                bf += '{:02x}'.format(k & 0xFF)
                checksum ^= k & 0xFF
            else:
                bf += '{:02x}'.format(k & 0xFF)
                checksum ^= k & 0xFF
                bf += '{:02x}'.format((k >> 8) & 0xFF)
                checksum ^= (k >> 8) & 0xFF

        bf += '{:02x}'.format(checksum)

        return bf

    def send_request_msp_set_raw_rc(self, channels):
        self.send_request_msp(self.create_packet_msp(MSP_SET_RAW_RC, channels))

    def send_request_msp_set_command(self, command_type):
        self.send_request_msp(self.create_packet_msp(MSP_SET_COMMAND, [command_type]))

    def send_request_msp_get_debug(self, requests):
        for i in range(len(requests)):
            self.send_request_msp(self.create_packet_msp(requests[i], []))

    def send_request_msp_set_acc_trim(self, trim_roll, trim_pitch):
        self.send_request_msp(self.create_packet_msp(MSP_ACC_TRIM, [trim_roll, trim_pitch]))

    def send_request_msp_acc_trim(self):
        if self.connected:
            self.send_request_msp(self.create_packet_msp(MSP_ACC_TRIM, []))
        else:
            print("Error: Not connected to server")

    def send_request_msp_eeprom_write(self):
        self.send_request_msp(self.create_packet_msp(MSP_EEPROM_WRITE, []))

    def send_request_msp_set_motor(self, motor_speeds):
        payload = []
        for speed in motor_speeds:
            payload.append(speed & 0xFF)
            payload.append((speed >> 8) & 0xFF)
        self.send_request_msp(self.create_packet_msp(MSP_SET_MOTOR, payload))

    def write_function(self):
        requests = [MSP_RC, MSP_ATTITUDE, MSP_RAW_IMU, MSP_ALTITUDE, MSP_ANALOG]
        self.send_request_msp_acc_trim()

        while self.connected:
            self.droneRC[:] = self.rc_values()
            self.send_request_msp_set_raw_rc(self.droneRC)
            self.send_request_msp_get_debug(requests)

            if self.command_type != self.NONE_COMMAND:
                self.send_request_msp_set_command(self.command_type)
                self.command_type = self.NONE_COMMAND

            time.sleep(0.022)

    # Sensor Data Retrieval
    def get_height(self):
        data = []
        self.create_packet_msp(MSP_ALTITUDE, data)
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 109:
                i += 1
            if i + 3 < len(data):
                height = self.read16(data[i + 1:i + 3])
                print(f"Height: {height} cm")
                return height

    def get_vario(self):
        data = []
        self.create_packet_msp(MSP_ALTITUDE, data)
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 109:
                i += 1
            if i + 7 < len(data):
                return self.read16(data[i + 5:i + 7])

    def get_roll(self):
        data = []
        self.create_packet_msp(MSP_ATTITUDE, data)
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 108:
                i += 1
            if i + 3 < len(data):
                roll = self.read16(data[i + 1:i + 3]) / 10
                print(f"Roll: {roll} degrees")
                return roll

    def get_pitch(self):
        data = []
        self.create_packet_msp(MSP_ATTITUDE, data)
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 108:
                i += 1
            if i + 5 < len(data):
                pitch = self.read16(data[i + 3:i + 5]) / 10
                print(f"Pitch: {pitch} degrees")
                return pitch

    def get_yaw(self):
        data = []
        self.create_packet_msp(MSP_ATTITUDE, data)
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 108:
                i += 1
            if i + 7 < len(data):
                return self.read16(data[i + 5:i + 7])

    def get_acc_x(self):
        data = []
        self.create_packet_msp(MSP_RAW_IMU, data)
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 102:
                i += 1
            if i + 3 < len(data):
                return self.read16(data[i + 1:i + 3])

    def get_acc_y(self):
        data = []
        self.create_packet_msp(MSP_RAW_IMU, data)
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 102:
                i += 1
            if i + 5 < len(data):
                return self.read16(data[i + 3:i + 5])

    def get_acc_z(self):
        data = []
        self.create_packet_msp(MSP_RAW_IMU, data)
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 102:
                i += 1
            if i + 7 < len(data):
                return self.read16(data[i + 5:i + 7])

    def get_gyro_x(self):
        data = []
        self.create_packet_msp(MSP_RAW_IMU, data)
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 102:
                i += 1
            if i + 9 < len(data):
                return self.read16(data[i + 7:i + 9])

    def get_gyro_y(self):
        data = []
        self.create_packet_msp(MSP_RAW_IMU, data)
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 102:
                i += 1
            if i + 11 < len(data):
                return self.read16(data[i + 9:i + 11])

    def get_gyro_z(self):
        data = []
        self.create_packet_msp(MSP_RAW_IMU, data)
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 102:
                i += 1
            if i + 13 < len(data):
                return self.read16(data[i + 11:i + 13])

    def get_mag_x(self):
        data = []
        self.create_packet_msp(MSP_RAW_IMU, data)
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 102:
                i += 1
            if i + 15 < len(data):
                return self.read16(data[i + 13:i + 15])

    def get_mag_y(self):
        data = []
        self.create_packet_msp(MSP_RAW_IMU, data)
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 102:
                i += 1
            if i + 17 < len(data):
                return self.read16(data[i + 15:i + 17])

    def get_mag_z(self):
        data = []
        self.create_packet_msp(MSP_RAW_IMU, data)
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 102:
                i += 1
            if i + 19 < len(data):
                return self.read16(data[i + 17:i + 19])

    def get_gps(self):
        data = []
        self.create_packet_msp(MSP_RAW_GPS, data)
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            # print(f"Received data: {data.hex()}")
            i = 0
            while i < len(data) and data[i] != 106:
                i += 1
            if i + 16 <= len(data):
                lat = self.read32(data[i + 1:i + 5])
                lon = self.read32(data[i + 5:i + 9])
                num_sat = self.read8(data[i+9])
                alt = self.read16(data[i + 10:i + 12])
                speed = self.read16(data[i + 12:i + 14])
                ground_course = self.read16(data[i + 14:i + 16])

                gps_data = {
                    "latitude": lat / 1e7,
                    "longitude": lon / 1e7,
                    "num_sat": num_sat,
                    "altitude": alt,
                    "speed": speed,
                    "ground_course": ground_course
                }

                print(f"GPS Data: {gps_data}")
                return gps_data
        print("Failed to get GPS data")
        return None

    def get_drone_pos(self):
        drone_pos = self.get_gps()
        if drone_pos:
            self.drone_position = [drone_pos["latitude"], drone_pos["longitude"], drone_pos["altitude"]]
        return self.drone_position

    def calibrate_acceleration(self):
        self.create_packet_msp(MSP_ACC_CALIBRATION, [])
        time.sleep(5)
        acc_x = self.get_acc_x()
        acc_y = self.get_acc_y()
        acc_z = self.get_acc_z()
        print("Calibrated Accelerometer Values:")
        print(f"X-Axis: {acc_x}, Y-Axis: {acc_y}, Z-Axis: {acc_z}")

    def calibrate_magnetometer(self):
        self.create_packet_msp(MSP_MAG_CALIBRATION, [])
        time.sleep(5)
        mag_x = self.get_mag_x()
        mag_y = self.get_mag_y()
        mag_z = self.get_mag_z()
        print("Calibrated Magnetometer Values:")
        print(f"X-Axis: {mag_x}, Y-Axis: {mag_y}, Z-Axis: {mag_z}")

    def get_battery(self):
        data = []
        self.create_packet_msp(MSP_ANALOG, data)
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 110:
                i += 1
            if i + 1 < len(data):
                battery_value = data[i + 1] / 10
                print(f"Battery: {battery_value} volts")
                return battery_value

    def get_battery_percentage(self):
        data = []
        self.create_send_msp_packet(MSP_ANALOG, data)
        for i in range(RETRY_COUNT):
            data = self.receive_packet()
            i = 0
            while i < len(data) and data[i] != 110:
                i += 1
            if i + 1 < len(data):
                battery_voltage = data[i + 1] / 10
                battery_percentage = (battery_voltage / 4.2) * 100
                print(f"Battery: {battery_percentage:.2f}%")
                return battery_percentage

    # Packet Reading Functions
    def send_packet(self, buff):
        self.client.send(buff)

    def receive_packet(self):
        return self.client.recv(4096)
    
    def read8(self, arr):
        return int.from_bytes(arr,byteorder='little',signed=False)
    
    def read16(self, arr):
        return int.from_bytes(arr, byteorder='little', signed=True)

    def read32(self, arr):
        return int.from_bytes(arr, byteorder='little', signed=True)

    # Position Hold Functions
    def activate_position_hold(self):
        self.target_gps = self.get_gps()
        if self.target_gps:
            self.position_hold_active = True
            print(f"Position hold activated at: {self.target_gps}")
        else:
            print("Failed to get GPS coordinates for position hold")

    def deactivate_position_hold(self):
        self.position_hold_active = False
        print("Position hold deactivated")

    def update_position_hold(self):
        if self.position_hold_active:
            current_gps = self.get_gps()
            if current_gps:
                error_lat = self.target_gps["latitude"] - current_gps["latitude"]
                error_lon = self.target_gps["longitude"] - current_gps["longitude"]

                P_lat = self.Kp_pos[0] * error_lat
                P_lon = self.Kp_pos[1] * error_lon

                self.error_sum_pos[0] += error_lat
                self.error_sum_pos[1] += error_lon
                I_lat = self.Ki_pos[0] * self.error_sum_pos[0]
                I_lon = self.Ki_pos[1] * self.error_sum_pos[1]

                D_lat = self.Kd_pos[0] * (error_lat - self.prev_error_pos[0])
                D_lon = self.Kd_pos[1] * (error_lon - self.prev_error_pos[1])

                control_lat = P_lat + I_lat + D_lat
                control_lon = P_lon + I_lon + D_lon

                self.prev_error_pos[0] = error_lat
                self.prev_error_pos[1] = error_lon

                self.rcPitch = int(1500 + control_lat * 1000)
                self.rcRoll = int(1500 + control_lon * 1000)

                self.rcPitch = self.clamp_rc(self.rcPitch)
                self.rcRoll = self.clamp_rc(self.rcRoll)

                self.send_request_msp_set_raw_rc(self.rc_values())
                print(f"Position hold: lat_error={error_lat:.6f}, lon_error={error_lon:.6f}, rcPitch={self.rcPitch}, rcRoll={self.rcRoll}")

            time.sleep(0.1)
