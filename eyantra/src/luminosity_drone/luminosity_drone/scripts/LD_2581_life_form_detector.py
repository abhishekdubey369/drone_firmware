#!/usr/bin/env python3
from swift_msgs.msg import *
from geometry_msgs.msg import PoseArray,Twist
from std_msgs.msg import Int16,Empty
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import numpy as np
import rospy
import time
import math
import cv2
from sensor_msgs.msg import Image,CompressedImage,CameraInfo
from luminosity_drone.msg import Biolocation
from theora_image_transport.msg import Packet
from cv_bridge import CvBridge
from imutils import contours
from skimage import measure
import numpy as np
import time
import imutils

global duration 
duration = 40
global pub
pub = 0

class LifeFormDetector:
    def __init__(self,dur):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/swift/camera_rgb/image_raw', Image, self.image_callback)
        self.biolocation_pub = rospy.Publisher('/astrobiolocation', Biolocation, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/drone/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(30)
        self.detected_organisms = []
        self.landing_completed = False
        self.swift_drone = SwiftDrone()
        global duration
        self.duration = dur  # Initialize Swift Drone here

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.detect_organisms(cv_image)
        except Exception as e:
            rospy.logerr(f"Error processing image: {str(e)}")

    def detect_organisms(self, image):
        num_leds, centroids = self.detect_leds(image)

        if num_leds >= 2 and num_leds <= 4:
            organism_type = self.determine_organism_type(num_leds)
            whycon_x, whycon_y = centroids[0]

            if not self.landing_completed and num_leds >= 2:
                self.duration = fly_in_circular_motion(self.swift_drone, radius=9.0, angular_speed=0.3)
                self.update_detected_organisms(organism_type, whycon_x, whycon_y)
        elif num_leds < 2 and self.duration > 0:
            print(self.duration)
            self.duration = fly_in_circular_motion(self.swift_drone, radius=9.0, angular_speed=0.3)
        else:
            self.land_drone()

    def update_detected_organisms(self, organism_type, whycon_x, whycon_y):
        if not self.detected_organisms or organism_type >= self.detected_organisms[0]['priority']:
            self.detected_organisms = [{
                'type': organism_type,
                'whycon_x': whycon_x,
                'whycon_y': whycon_y,
                'priority': organism_type
            }]
            rospy.loginfo(f"Detected Organism: {organism_type}")

    def detect_leds(self, image):
        # Convert the image to the HSV color space for better color segmentation
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply GaussianBlur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), cv2.BORDER_DEFAULT)
        _, binary_mask = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # Connected component analysis
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_mask, connectivity=8)

        # Ensure there are 2, 3, or 4 LEDs
        valid_centroids = []

        # Group centroids by distance
        centroid_groups = self.group_centroids_by_distance(centroids)

        # Find the group with the highest priority (4 LEDs > 3 LEDs > 2 LEDs)
        for group_size in [4, 3, 2]:
            closest_group = self.find_closest_group(centroid_groups, group_size)
            if closest_group:
                # Draw the bright spots on the image
                for centroid in closest_group:
                    cv2.circle(image, (int(centroid[0]), int(centroid[1])), 4, (0, 255, 0), -1)
                    valid_centroids.append((int(centroid[0]), int(centroid[1])))

                print(f"{len(valid_centroids)}")

                cv2.imwrite(f"image{len(valid_centroids)}.jpg", image)

                break  # Break after considering the highest priority group

        return len(valid_centroids), valid_centroids

    def group_centroids_by_distance(self, centroids):
        centroid_groups = []

        for centroid in centroids:
            # Check if the centroid is close to any existing group
            matched_group = None
            for group in centroid_groups:
                if any(self.distance(centroid, group_centroid) < 13 for group_centroid in group):
                    matched_group = group
                    break

            # If close to a group, add to that group; otherwise, create a new group
            if matched_group:
                matched_group.append(centroid)
            else:
                centroid_groups.append([centroid])

        return centroid_groups

    def find_closest_group(self, groups, target_size):
        closest_group = None
        closest_distance = float('inf')

        for group in groups:
            if len(group) == target_size:
                # Calculate the average distance between the centroids in the group
                avg_distance = np.mean([self.distance(group[i], group[j]) for i in range(target_size) for j in range(i + 1, target_size)])

                if avg_distance < closest_distance:
                    closest_group = group
                    closest_distance = avg_distance

        return closest_group

    @staticmethod
    def distance(point1, point2):
        return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

 
    def determine_organism_type(self, num_leds):
        organism_types = {2: 'alien_a', 3: 'alien_b', 4: 'alien_c'}
        return organism_types.get(num_leds, 'unknown')

    def publish_biolocation(self, organism_type, whycon_x, whycon_y):
        biolocation_msg = Biolocation()
        biolocation_msg.organism_type = organism_type
        biolocation_msg.whycon_x = whycon_x
        biolocation_msg.whycon_y = whycon_y
        biolocation_msg.whycon_z = 0.0
        self.biolocation_pub.publish(biolocation_msg)


    def land_drone(self):
        if self.detected_organisms:
            highest_priority_organism = self.detected_organisms[0];global pub
            if pub==0:
                self.publish_biolocation(
					highest_priority_organism['type'],
                	highest_priority_organism['whycon_x'],
                	highest_priority_organism['whycon_y']);pub=1
            rospy.loginfo(f"Landing drone for the highest priority organism: {highest_priority_organism['type']}")
        else:
            rospy.loginfo("No organisms detected. Landing drone.")
        # Implement your landing logic here
        target_position = [11, 11, 37];self.swift_drone.pid(target_position)
        # landing_cmd = Twist()
        # landing_cmd.linear.z = -0.5
        # self.cmd_vel_pub.publish(landing_cmd)
        self.landing_completed = True


class SwiftDrone:

	def __init__(self):
		# rospy.init_node('life_form')
		self.start_time = rospy.get_time()
		self.rate = rospy.Rate(30)
		self.drone_position = [0.0,0.0,0.0]	
		self.cmd = swift_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500
		self.Kp = [30,30,40]
		self.Ki = [0,0,0.00018]	
		self.Kd = [15000,150000,150000];self.Kp_altitude = 40;self.Ki_altitude = 0.00018;self.Kd_altitude = 150000;self.prev_altitude_error = 0.0;self.altitude_integral = 0.0
		# self.allpoint = [[0, 0, 23],[2, 0, 23],[2, 2, 23],[2, 2, 25],[-5, 2, 22.4],[-5, -3, 23],[-5, -3, 19],[7, -3, 19],[7, 0, 18],[0, 0, 16]]
		# self.setpoint = self.allpoint[0]
		self.i = 1
		self.abserror = [0,0,0]

		self.sample_time = 60
		self.prev_values = [0,0,0]
		self.max_values = 2000
		self.min_values = 1000
		self.error = [0,0,0]
		self.now=0.0000
		self.timechange=0.000
		self.errsum=[0,0,0]
		self.derr=[0,0,0]
		self.last_time=0.0000
		self.out_roll=0.000
		self.out_pitch=0.000
		self.out_throttle=0.000
		self.radius = 7.5
		self.angular_speed = 0.5
		self.move_cmd = Twist()
		self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size=1)
		self.alt_error_pub = rospy.Publisher('/alt_error',Float64,queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error',Float64,queue_size=1)
		self.roll_error_pub = rospy.Publisher('/roll_error',Float64,queue_size=1)
		self.drone_cmd_publisher = rospy.Publisher('/drone/cmd_vel', Twist, queue_size=1);self.cmd_vel_pub = rospy.Publisher('/drone/cmd_vel', Twist, queue_size=1);self.altitude_error_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		# self.arm() # ARMING THE DRONE
		
	def move_drone(self,centroid, frame_size):
		self.proportional_gain = 0.01
		self.offset_x = centroid[0] - frame_size[0] / 2
		self.offset_y = centroid[1] - frame_size[1] / 2
		self.move_cmd = Twist()
		self.move_cmd.linear.x = self.proportional_gain * self.offset_x
		self.move_cmd.linear.y = self.proportional_gain * self.offset_y
		
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)

	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)
		# self.update_setpoint()
		# ImageCapture()
		rospy.sleep(1)

	def update_setpoint(self, radius, angular_speed):
		time_since_start = rospy.Time.now().to_sec()
		setpoint = [
    		int(radius * math.cos(angular_speed * time_since_start)),
    		int(radius * math.sin(angular_speed * time_since_start)),
    		int(22.0)
		]
		self.pid(setpoint)

	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z
		global pub
		if pub==0:
			self.update_setpoint(radius=9.0,angular_speed=0.3)

	def pid(self, setpoint):
		# time functions
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

				# Integration for Ki
				self.errsum[0] = self.errsum[0] + (self.error[0] * self.timechange)
				self.errsum[1] = self.errsum[1] + (self.error[1] * self.timechange)
				self.errsum[2] = self.errsum[2] + (self.error[2] * self.timechange)

				# Derivation for Kd
				self.derr[0] = (self.error[0] - self.prev_values[0]) / self.timechange
				self.derr[1] = (self.error[1] - self.prev_values[1]) / self.timechange
				self.derr[2] = (self.error[2] - self.prev_values[2]) / self.timechange

				# Calculating output in 1500
				self.cmd.rcRoll = int(1500 - (self.Kp[0] * self.error[0]) - (self.Kd[0] * self.derr[0]))
				self.cmd.rcPitch = int(1500 + (self.Kp[1] * self.error[1]) + (self.Kd[1] * self.derr[1]))
				self.cmd.rcThrottle = int(1500 + (self.Kp[2] * self.error[2]) + (self.Kd[2] * self.derr[2]) - (self.errsum[2] * self.Ki[2]))

				# Checking min and max threshold and updating on true
				# Throttle Conditions
				if self.cmd.rcThrottle > 2000:
					self.cmd.rcThrottle = self.max_values
				if self.cmd.rcThrottle < 1000:
					self.cmd.rcThrottle = self.min_values

				# Pitch Conditions
				if self.cmd.rcPitch > 2000:
					self.cmd.rcPitch = self.max_values
				if self.cmd.rcPitch < 1000:
					self.cmd.rcPitch = self.min_values

				# Roll Conditions
				if self.cmd.rcRoll > 2000:
					self.cmd.rcRoll = self.max_values
				if self.cmd.rcRoll < 1000:
					self.cmd.rcRoll = self.min_values

				# Publishing values on topic 'drone command'
				self.command_pub.publish(self.cmd)

				# Updating prev values for all axis
				self.prev_values[0] = self.error[0]
				self.prev_values[1] = self.error[1]
				self.prev_values[2] = self.error[2]

			# Updating last time value
			self.last_time = self.now

		self.altitude_error_pub.publish(self.cmd.rcThrottle)
		self.command_pub.publish(self.cmd)
		self.alt_error_pub.publish(self.error[2])
		self.pitch_error_pub.publish(self.error[1])
		self.roll_error_pub.publish(self.error[0])
		self.drone_cmd_publisher.publish(self.move_cmd);


def fly_in_circular_motion(swift, radius, angular_speed):
    start_time = rospy.Time.now().to_sec()
    current_time = rospy.Time.now().to_sec();global duration

    if duration>0:
        swift.update_setpoint(radius, angular_speed)
        rospy.sleep(0.06)
        current_time = rospy.Time.now().to_sec();duration=duration-0.06

    return duration
	


if __name__ == '__main__':
    try:
        rospy.init_node('life_form');print('run successfully')
        LifeFormDetector(duration)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass