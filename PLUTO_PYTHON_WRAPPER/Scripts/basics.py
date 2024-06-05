import cv2
import numpy as np
import subprocess
import threading
from plutocontrol import pluto
import time
import os
from cvzone.FaceDetectionModule import FaceDetector


# Initialize the drone object
drone = pluto()
drone.cam()
drone.connect()

try:
    drone.arm()
    
    # Start the timer
    start_time = time.time()
    while True:
        # Send the PID command to the drone
        drone.left()
        time.sleep(15)
        
        # Update the elapsed time
        elapsed_time = time.time() - start_time
        if elapsed_time >= 4 * 60:
            break
except KeyboardInterrupt:
    # Handle user interruption
    pass
finally:
    # Ensure the drone is disarmed and disconnected properly
    drone.disarm()
    drone.disconnect()
