#!/usr/bin/env python

'''
* Team Id : 					HB#1038
* Author List : 				ALOK SENAPATI, ASHIS KUMAR MAHARANA
* Filename : 					1038_pos_hold.py
* Theme : 						Hungry Bird(HB)
* Functions : 					disarm(), arm(), whycon_callback(), pid()
* Global Variables : 			None
'''

# Importing the required libraries

from plutodrone.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
from plutodrone.srv import *
import rospy
from time import time
from traceback import print_exc
from collections import deque
import math


class Pluto():

	peak_amplitude_tolerance = 0.05
	state_autotune_off = 'off'
	state_step_up = 'output step up'
	state_step_down = 'output step down'
	state_succeeded = 'SUCCESS'
	state_failed = 'FAILED'

	tuning_rule = {
		"ziegler-nichols": [34, 40, 160]
	}

	"""docstring for Edrone"""
	def __init__(self):

		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [0.0,0.0,25.0]

		#---------------------Declaring a cmd of message type PlutoMsg and initializing values----------------
		self.cmd = PlutoMsg()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500
		# self.cmd.plutoIndex = 0
		self.sample_time = 0.060              # in seconds
		self.is_whycon_detected = False
		self.time = time
		# ----------------------------------------------------------------------------------------------------

		#----------------------PID parameters------------------------------------------------------------------
		self.Kp = [0.00, 0.00, 0.00]
		self.Ki = [0.00, 0.00, 0.00]
		self.Kd = [0.00, 0.00, 0.00]
		# self.Kp = [7.13, -6.999, 21.14]
		# self.Ki = [0.0455, -0.0855, 0.0055]
		# self.Kd = [17.83, -19.76, 1.29]

		#-----------------------Other required variables for pid -----------------------------------------------

		self.errors = [0.0, 0.0, 0.0]
		self.error_sum = [0.0, 0.0, 0.0]
		self.prev_position = [0.0, 0.0, 0.0]
		self.max_values = [1800, 1800, 1800]
		self.min_values = [1200, 1200, 1200]
		self.update_value = [0.0, 0.0, 0.0]

		# these terms were published to debug while adjusting the gain
		self.P_term = [0.0, 0.0, 0.0]
		self.D_term = [0.0, 0.0, 0.0]
		self.I_term = [0.0, 0.0, 0.0]
		# --------------------------------------------------------------------------------------------------------

		#--------------------Required parameters for Ziegler-Nichols tuning-----------------------------------
		# Finding th ultimate Ku and Pu
		self.inputs = deque(maxlen=round(1 / self.sample_time))
		self.update_step = 0.5
		self.noise_band = 0.05
		self.state = Pluto.state_autotune_off
		self.peak_timestamps = deque(maxlen=5)
		self.peaks = deque(maxlen=5)           # Keep track of the peak errors of each cycle while oscillation
		self.last_timestamp = 0
		self.tuning = [False, False, True]
		self.tuned = [False, False, False]
		self.peak_type = 0
		self.peak_count = 0
		self.last_update_value = 0
		self.induced_amplitude = 0
		self.Ku = 0
		self.Pu = 0
		# ----------------------------------------------------------------------------------------------------

		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error, /yaw_error
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
		self.alt_error_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)
		self.p_pub = rospy.Publisher('/P_term', Float64, queue_size=1)
		self.d_pub = rospy.Publisher('/D_term', Float64, queue_size=1)
		self.i_pub = rospy.Publisher('/I_term', Float64, queue_size=1)

		#-----------------------------------------------------------------------------------------------------------

		# Subscribing to whycon/poses
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)

		self.disarm()
		self.arm() # ARMING THE DRONE



	'''
	* Function Name : 		disarm
	* Input : 				Instance of the class calling it
	* Output : 				None
	* Logic : 				This function sets the value of rcAUX4 below its lower limit and this disarms the drone
	* Example Call : 		drone.disarm()   drone is the object of type Pluto
	'''

	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	'''
	* Function Name : 		arm
	* Input : 				Instance of the class calling it
	* Output : 				None
	* Logic : 				This function sets the value of drone parameter to stable value and set throttle to minimum and this arms the drone
	* Example Call : 		drone.arm()   drone is the object of type Pluto
	'''

	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	                # Publishing /drone_command
		rospy.sleep(1)


	'''
	* Function Name : 		whycon_callback
	* Input : 				Instance of the class calling it and msg from whycon/poses
	* Output : 				None
	* Logic : 				This  callback function gets the whycon coordinates from the message published by whycon/poses topic
							and it is identified as the position of the drone
	* Example Call : 		drone.whycon_callback(message)   drone is the object of type Pluto
	'''

	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses
	def whycon_callback(self,msg):
		self.is_whycon_detected = True		                 # Because the callback is called only when the whycon is detected.
		self.drone_position[0] = msg.poses[0].position.x
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z


	# auto-tune function
	def tune(self):
		for i in range(len(self.tuning)):
			if self.tuning[i] and not self.tuned[i]:
				input = self.drone_position[i]
				# Run it periodically
				now = self.time
				if self.state == Pluto.state_autotune_off or self.state == Pluto.state_succeeded or self.state == Pluto.state_failed:
					self.reset(now)
				elif ( now - self.last_timestamp ) < self.sample_time :
					return False
				self.last_timestamp = now

				# Decide whether to go up or down in update value
				if self.state == Pluto.state_step_up and (input > self.setpoint[i] + self.noise_band):
					self.state = Pluto.state_step_down

				elif self.state == Pluto.state_step_down and (input < self.setpoint[i] - self.noise_band):
					self.state = Pluto.state_step_up
					
				# Calculate the update value
				if self.state == Pluto.state_step_up:
					self.update_value[i] += self.update_step
				elif self.state == Pluto.state_step_down:
					self.update_value[i] -= self.update_step

				# Identify the peaks
				is_max = True
				is_min = True
				for val in self.inputs:
					is_max = is_max and (input >= val)
					is_min = is_min and (input <= val)
				self.inputs.append(input)

				# Don't trust the inputs until it is full
				if len(self.inputs) < self.inputs.maxlen:
					return False

				# Change of state
				inflection = False

				if is_max:
					# In positive cycle
					if self.peak_type == -1:
						inflection = True
					self.peak_type = 1
				elif is_min:
					# In negative cycle
					if self.peak_type == 1:
						inflection = True
					self.peak_type = -1

				# Keep track of peaks and peaks' timestamp
				if inflection:
					self.peak_count += 1
					self.peaks.append(input)
					self.peak_timestamps.append(now)

				# Check for convergence in drone position
				# convergence of peaks assesed on last 4 peaks
				self.induced_amplitude = 0

				if inflection and (self.peak_count > 4):
					# Middle value
					abs_max = self.peaks[-2]
					abs_min = self.peaks[-2]

					for j in range(0, len(self.peaks) - 2):
						self.induced_amplitude += abs(self.peaks[j] - self.peaks[j+1])
						abs_max = max(self.peaks[j], abs_max)
						abs_min = min(self.peaks[j], abs_min)
					self.induced_amplitude /= 6.0
				
					# Check covergence criterion for induced oscillation
					amplitude_deviation = ((0.5 * (abs_max - abs_min) - self.induced_amplitude) / self.induced_amplitude)

					if amplitude_deviation < Pluto.peak_amplitude_tolerance:
						self.state = Pluto.state_succeeded

				# If did not succeeded in auto tune, terminate after 10 cycles
				if self.peak_count >= 20:
					self.update_value[i] = 0
					self.state = Pluto.state_failed
					return True

				if self.state == Pluto.state_succeeded:
					self.update_value[i] = 0

					self. Ku = 4.0 * self.update_step / (self.induced_amplitude * math.pi)

					period1 = self.peak_timestamps[3] - self.peak_timestamps[1]
					period2 = self.peak_timestamps[4] - self.peak_timestamps[2]
					self.Pu = 0.5 * (period1 + period2)

					if i == 2:
						# Altitude tuning
						print("Altitude: " + str(self.Ku))
						self.setPidParams(i, self.Ku, self.Pu)
						self.tuned[2] = True
						self.tuning[2] = False
						self.tuning[0] = True
					elif i == 0:
						# Roll tuning
						print("Roll: " + str(self.Ku))
						# self.period = 2 * (self.end - self.start)
						self.setPidParams(i, self.Ku, self.Pu)
						self.tuned[0] = True
						self.tuning[0] = False
						self.tuning[1] = True
					elif i == 1:
						# Pitch tuning
						print("Pitch: " + str(self.Ku))
						# self.period = 2 * (self.end - self.start)
						self.setPidParams(i, self.Ku, self.Pu)
						self.tuned[1] = True
						self.tuning[1] = False
					self.reset(now)
					return True
				return False


	def setPidParams(self, i, ku, pu):
		divisors = Pluto.tuning_rule['ziegler-nichols']
		self.Kp[i] = ku / divisors[0]
		self.Kd[i] = self.Kp[i] * (pu / divisors[2])
		self.Ki[i] = self.Kp[i] / (pu / divisors[1])
		print(str(self.Kp[i]) + " " + str(self.Kd[i]) + " " + str(self.Kp[i]))

	def reset(self, timestamp):
		self.peaks.clear()
		self.peak_type = 0
		self.peak_count = 0
		self.update_value = [0.0, 0.0, 0.0]
		self.Ku = 0
		self.Pu = 0
		self.inputs.clear()
		self.peak_timestamps.clear()
		self.peak_timestamps.append(timestamp)
		self.state = Pluto.state_step_up

	'''
	* Function Name : 		pid
	* Input : 				Instance of the class calling it
	* Output : 				None
	* Logic : 				This function calculates error and calculates the update in drone command values
							implementing PID control using Kp,Ki,Kd and sample_time and publishes to the drone command
	* Example Call : 		drone.pid()   drone is the object of type Pluto
	'''
	def pid(self):
		now = time

		# this is used to tune the pid in regular intervals of 60ms
		if (now - self.last_timestamp) < self.sample_time:
			return

		for	i in range(len(self.errors)):
			self.errors[i] = self.setpoint[i] - self.drone_position[i]

			if abs(self.update_value[i]) < self.max_values[i] - 1500:
				self.error_sum[i] += self.Ki[i] * self.errors[i]
				if self.errors[i] == 0:
					self.error_sum[i] = 0
				self.I_term[i] = max(min(self.error_sum[i], 1000), -1000) * self.sample_time

			self.P_term[i] = self.errors[i] * self.Kp[i]
			self.D_term[i] = - ((self.drone_position[i] - self.prev_position[i]) * self.Kd[i] / self.sample_time)
			
			self.update_value[i] = self.P_term[i] + self.D_term[i] + self.I_term[i]

			# To maintain the stable values in all the axis if the error box is less than the limit
			if abs(self.errors[i]) > self.noise_band:
				self.update_value[i] = self.P_term[i] + self.D_term[i] + self.I_term[i]
			else:
				self.update_value[i] = 0.00

			# keeping track of some values
			self.prev_position[i] = self.drone_position[i]

		self.last_timestamp = now

		#------------------------------------------------------------------------------------------------------------------------

	def publish_data(self):
		self.pitch_error_pub.publish(self.errors[1])
		self.roll_error_pub.publish(self.errors[0])
		self.alt_error_pub.publish(self.errors[2])

		# # published to debug
		# self.p_pub.publish(self.P_term[i])
		# self.d_pub.publish(self.D_term[i])
		# self.i_pub.publish(self.I_term[i])

		if self.is_whycon_detected:
			# limiting the output in between the max and min
			self.cmd.rcRoll = min(max(1500 + self.update_value[0], self.min_values[0]), self.max_values[0])
			self.cmd.rcPitch = min(max(1500 + self.update_value[1], self.min_values[1]), self.max_values[1])
			self.cmd.rcThrottle = min(max(1500 - self.update_value[2], self.min_values[2]), self.max_values[2])


		# Drone needs to be controlled even when the whycon is not detected
		# This helps in two of the cases
		# First one in which the drone is out of range of the camera
		# Second one in which within the arena, for any reason, if the whycon is not detected
		# In both of the above mentioned cases to prevent the absurd behaviour of drone
		# the pitch, roll and throttle are controlled in such a way that the drone follows the reverse of the path it has been following previously
		# For example in pitch, if the value is greater than 1500, it is moving forward and goes off the camera so it needs to come back to be visible again
		# so we have given a pitch value less than 1500
		# Similar for all other axes, for Throttle we are giving a value such that it always goes down to the floor when undetected

		else:
			# If whycon not detected do this...
			self.cmd.rcPitch = 1450 if self.cmd.rcPitch > 1500 else 1550
			self.cmd.rcRoll = 1450 if self.cmd.rcRoll > 1500 else 1550
			self.cmd.rcThrottle = 1400 if self.cmd.rcThrottle > 1200 else 1000
		
		
		self.is_whycon_detected = False			# Irrespective of the sample time in each loop the whycon detection is set to False
												# It is set to True only when the whycon is detected

		self.command_pub.publish(self.cmd)



if __name__ == '__main__':

	e_drone = Pluto()

	while not rospy.is_shutdown():
		try:
			if any(e_drone.tuning):
				e_drone.tune()
			else:
				e_drone.pid()
			e_drone.publish_data()

		except Exception as e:
			# to traceback the error
			print e
			print_exc()
