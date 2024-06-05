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
import imutils


class ImageCapture:
    def __init__(self):
        self.bridge = CvBridge()
        self.img_why = rospy.Subscriber('/whycon/image_out', Image, self.img_whycon)
        self.image_com = rospy.Subscriber('/swift/camera_rgb/image_raw/compressed', CompressedImage, self.com_image)
        self.image_sub = rospy.Subscriber('/swift/camera_rgb/image_raw', Image, self.image_callback)
        self.camera_info = rospy.Subscriber('/swift/camera_rgb/camera_info', CameraInfo, self.cam_info)
        self.biolocation_pub = rospy.Publisher('/astrobiolocation', Biolocation, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/drone/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(10)  # Adjust the rate as needed
        self.target_organism = None
        self.detected_organisms = []

    def img_whycon(self, msg):
        try:
            pass
        except Exception as e:
            rospy.logerr(f"Error processing image: {str(e)}")

    def show_image(self, img):
        cv2.imshow("Image Window", img)
        cv2.waitKey(3)

    def cam_info(self, msg: CameraInfo):
        pass

    def com_image(self, msg):
        try:
            pass
        except Exception as e:
            rospy.logerr(f"Error in com: {str(e)}")

    def detect_leds(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), cv2.BORDER_DEFAULT)
        _, binary_mask = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_mask, connectivity=8)

        valid_centroids = []

        for i in range(1, num_labels):
            if stats[i, cv2.CC_STAT_AREA] > 10:
                cx, cy = centroids[i]
                centroid = (int(cx), int(cy))

                contour_mask = (labels == i).astype(np.uint8)
                contours, _ = cv2.findContours(contour_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                if contours:
                    perimeter = cv2.arcLength(contours[0], closed=True)
                    circularity = (4 * np.pi * stats[i, cv2.CC_STAT_AREA]) / (perimeter ** 2)

                    if 0.5 <= circularity <= 1.5:
                        cv2.circle(image, centroid, 4, (0, 255, 0), -1)
                        valid_centroids.append(centroid)

        cv2.imshow("LED Detection", image)
        cv2.waitKey(1)

        print(f"{len(valid_centroids)}")

        return len(valid_centroids), valid_centroids

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.detect_organisms(cv_image)
        except Exception as e:
            rospy.logerr(f"Error processing image: {str(e)}")

    def detect_organisms(self, image):
        num_leds, centroids = self.detect_leds(image)

        if num_leds > 1:
            organism_type = self.determine_organism_type(num_leds)
            whycon_x, whycon_y = centroids[0]
            self.publish_biolocation(organism_type, whycon_x, whycon_y)

            if organism_type == self.target_organism:
                self.land_drone()

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
        target_position = [11, 11, 37]
        landing_cmd = Twist()
        landing_cmd.linear.z = -0.5
        self.cmd_vel_pub.publish(landing_cmd)

class swift:
	"""docstring for swift"""
	def __init__(self):
		
		rospy.init_node('life_form')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint]
        # self.allpoint = [[0, 0, 23],[2, 0, 23],[2, 2, 23],[2, 2, 25],[-5, 2, 25],[-5, -3, 25],[-5, -3, 21],[7, -3, 21],[7, 0, 21],[0, 0, 19]]
		# self.setpoint = [2.05,2.00,17.20] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly


		#Declaring a cmd of message type swift_msgs and initializing values
		self.cmd = swift_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500


		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters

		self.Kp = [30,30,40]
		self.Ki = [0,0,0.00018]	
		self.Kd = [15000,150000,150000]
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



		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size=1)
		self.alt_error_pub = rospy.Publisher('/alt_error',Float64,queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error',Float64,queue_size=1)
		self.roll_error_pub = rospy.Publisher('/roll_error',Float64,queue_size=1)
		self.drone_cmd_publisher = rospy.Publisher('/drone/cmd_vel', Twist, queue_size=1)

		#------------------------Add other ROS Publishers here-----------------------------------------------------






	#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------




		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE
		
	def move_drone(self,centroid, frame_size):
		self.proportional_gain = 0.01
		self.offset_x = centroid[0] - frame_size[0] / 2
		self.offset_y = centroid[1] - frame_size[1] / 2
		self.move_cmd = Twist()
		self.move_cmd.linear.x = self.proportional_gain * self.offset_x
		self.move_cmd.linear.y = self.proportional_gain * self.offset_y
		
	def update_setpoint(self):
		time_since_start = (rospy.get_time() - self.start_time)
		self.setpoint = [
			self.radius * math.cos(self.angular_speed * time_since_start),
			self.radius * math.sin(self.angular_speed * time_since_start),
			22.0  # Maintain the same altitude for simplicity, adjust as needed
			]# Disarming condition of the drone
		

	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)
		self.start_time = rospy.get_time()# Publishing /drone_command
		# self.update_setpoint()
		# ImageCapture()
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z
		# print(msg)
		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------



	
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.Ki * 0.0008
		self.Kd[2] = alt.Kd * 0.3
		
	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------
	def pitch_set_pid(self,pitch):
		self.Kp[1] = pitch.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[1] = pitch.Ki * 0.0008
		self.Kd[1] = pitch.Kd * 0.3
		
	def roll_set_pid(self,roll):
		self.Kp[0] = roll.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[0] = roll.Ki * 0.0008
		self.Kd[0] = roll.Kd * 0.3		



	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------
		
		
		#time functions
		self.now = int(round(time.time() * 1000))
		self.timechange=self.now-self.last_time
		
		#delta time must be more than step time of Gazebo, otherwise same values will be repeated 
		if (self.timechange>self.sample_time):
			if (self.last_time!=0):			

				#Getting error of all coordinates		
				self.error[0]=self.drone_position[0] - self.setpoint[0]
				self.error[1]=self.drone_position[1] - self.setpoint[1]
				self.error[2]=self.drone_position[2] - self.setpoint[2]

				self.abserror[0] = self.error[0]
				self.abserror[1] = self.error[1]
				self.abserror[2] = self.error[2]
				# if(self.setpoint[2]<20 or self.setpoint[1]>8 or self.setpoint[1]<-8):
				# 	self.setpoint = [0,0,28]
				# if(self.abserror[2]>=10 and self.abserror[2]<=12):
				# print(self.setpoint)
				# print(self.abserror)
				ImageCapture()
				self.update_setpoint()
				
				
				# 	# self.setpoint = self.allpoint[self.i]
				# 	self.i = self.i + 1
				# if(self.abserror[0]<=0.2 and self.abserror[1]<=0.2 and self.abserror[2]>=2 and self.abserror[2]<4 and self.i<10):
				# 	print(self.setpoint)
				# 	self.update_setpoint()
				# 	self.i = self.i + 1
				
				

				#Integration for Ki
				self.errsum[0]=self.errsum[0]+(self.error[0]*self.timechange)
				self.errsum[1]=self.errsum[1]+(self.error[1]*self.timechange)
				self.errsum[2]=self.errsum[2]+(self.error[2]*self.timechange)


				#Derivation for Kd
				self.derr[0]=(self.error[0]-self.prev_values[0])/self.timechange
				self.derr[1]=(self.error[1]-self.prev_values[1])/self.timechange
				self.derr[2]=(self.error[2]-self.prev_values[2])/self.timechange


				#Calculating output in 1500
				self.cmd.rcRoll=int(1500-(self.Kp[0]*self.error[0])-(self.Kd[0]*self.derr[0]))
				self.cmd.rcPitch=int(1500+(self.Kp[1]*self.error[1])+(self.Kd[1]*self.derr[1]))
				self.cmd.rcThrottle=int(1500+(self.Kp[2]*self.error[2])+(self.Kd[2]*self.derr[2])-(self.errsum[2]*self.Ki[2]))

				
				#Checking min and max threshold and updating on true
				#Throttle Conditions
				if self.cmd.rcThrottle>2000:
					self.cmd.rcThrottle=self.max_values
				if self.cmd.rcThrottle<1000:
					self.cmd.rcThrottle=self.min_values		

				#Pitch Conditions
				if self.cmd.rcPitch>2000:
					self.cmd.rcPitch=self.max_values	
				if self.cmd.rcPitch<1000:
					self.cmd.rcPitch=self.min_values

				#Roll Conditions
				if self.cmd.rcRoll>2000:
					self.cmd.rcRoll=self.max_values
				if self.cmd.rcRoll<1000:
					self.cmd.rcRoll=self.min_values


				#Publishing values on topic 'drone command'
				self.command_pub.publish(self.cmd)

				
				#Updating prev values for all axis
				self.prev_values[0]=self.error[0]
				self.prev_values[1]=self.error[1]
				self.prev_values[2]=self.error[2]
		

		 		
		 	#Updating last time value	
			self.last_time=self.now


	#------------------------------------------------------------------------------------------------------------------------
		self.command_pub.publish(self.cmd)
		self.alt_error_pub.publish(self.error[2])
		self.pitch_error_pub.publish(self.error[1])
		self.roll_error_pub.publish(self.error[0])
		self.drone_cmd_publisher.publish(self.move_cmd)


if __name__ == '__main__':

	swift_drone = swift()
	r = rospy.Rate(30) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		swift_drone.pid()
		r.sleep()