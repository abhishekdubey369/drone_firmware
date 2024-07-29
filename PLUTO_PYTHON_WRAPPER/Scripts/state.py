import time
from Pluto import pluto
# from keyboard import keyboa

# drone = pluto()
# drone.connect()


def test_position_hold():
    # Initialize the pluto class
    my_drone = pluto()
    # my_drone = pluto()

    # Assuming you've instantiated your drone object as 'drone'
    # drone.connect()

    my_drone.connect()

    # my_drone.get_battery()
    # Give some time for the connection to establish
    time.sleep(2)

    # Arm the drone and take off
    my_drone.arm()
    time.sleep(5)
    my_drone.take_off()

    # Activate position hold
    my_drone.activate_position_hold()

    # Update position hold in a loop
    try:
        while my_drone:
            my_drone.update_position_hold()
            time.sleep(0.1)  # Adjust update rate as needed
    except KeyboardInterrupt:
        # Deactivate position hold and land the drone when stopping the script
        my_drone.deactivate_position_hold()
        my_drone.land()

    # Disconnect from the drone
    my_drone.disarm()
    time.sleep(5)
    my_drone.disconnect()

# Run the test function
try:
    test_position_hold()
except KeyboardInterrupt:
    exit

def continuously_get_sensor_values(interval_seconds=1):
    while True:
        
        # mag_x = drone.get_mag_x()
        # mag_y = drone.get_mag_y()
        # mag_z = drone.get_mag_z()
        # gyro_x = drone.get_gyro_x()
        # gyro_y = drone.get_gyro_y()
        # gyro_z = drone.get_gyro_z()
        # acc_x = drone.get_acc_x()
        # acc_y = drone.get_acc_y()
        # acc_z = drone.get_acc_z()
        gps = drone.get_gps()

        # print(f"Magnetometer (X,Y,Z): ({mag_x}, {mag_y}, {mag_z})")
        # print(f"Gyroscope (X,Y,Z): ({gyro_x}, {gyro_y}, {gyro_z})")
        # print(f"Accelerometer (X,Y,Z): ({acc_x}, {acc_y}, {acc_z})")

        # time.sleep(interval_seconds)

# Call the function to start retrieving sensor values continuously
# continuously_get_sensor_values()