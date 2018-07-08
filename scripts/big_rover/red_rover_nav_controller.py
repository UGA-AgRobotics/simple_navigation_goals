#!/usr/bin/env python

import roslib
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float64
from simple_navigation_goals.srv import *
from math import radians, copysign, sqrt, pow, pi, degrees
import PyKDL
import utm
import json



# Global settings:
rate = 10  # Turning and driving loop rate for red rover
angular_tolerance = rospy.get_param("~angular_tolerance", radians(2)) # degrees to radians




class NavController:

	def __init__(self):

		rospy.init_node('red_rover_nav_controller')

		# Subscribers:
		# rospy.Subscriber("/at_flag", Bool, self.flag_callback)  # sub to /at_flag topic from jackal_flags_node.py
		# rospy.Subscriber("/rf_stop", Bool, self.rf_stop_callback, queue_size=1)
		rospy.Subscriber("/driver/encoder_velocity", Float64, self.rover_velocity_callback)
		rospy.Subscriber("/driver/pivot", Float64, self.rover_pivot_callback, queue_size=1)
		rospy.Subscriber("/driver/run_throttle_test", Bool, self.throttle_test_callback)
		# rospy.Subscriber('/stop_rover', Bool, self.stop_rover_callback)
		# rospy.Subscriber('/start_rover', Bool, self.start_rover_callback)


		# Publishers:
		self.actuator_pub = rospy.Publisher('/driver/linear_drive_actuator', Float64, queue_size=1)  # TODO: double check queue sizes..
		self.throttle_pub = rospy.Publisher('/driver/throttle', UInt8, queue_size=1)  # TODO: double check queue sizes..
		self.articulator_pub = rospy.Publisher('/driver/articulation_relay', Float64, queue_size=1)  # TODO: double check queue sizes..
		# self.sample_publisher = rospy.Publisher('collect_sample', Bool, queue_size=1)


		# Establishing ROS service connections
		#############################################################################
		print("Waiting for start_sample_collection service..")
		rospy.wait_for_service('start_sample_collection')
		self.start_sample_collection = rospy.ServiceProxy('start_sample_collection', SampleCollection)
		print("start_sample_collection service ready.")

		print("Waiting for get_jackal_pos service..")
		rospy.wait_for_service('get_jackal_pos')
		self.get_jackal_pos = rospy.ServiceProxy('get_jackal_pos', JackalPos)
		print("get_jackal_pos service ready.")

		print("Waiting for get_jackal_rot service..")
		rospy.wait_for_service('get_jackal_rot')
		self.get_jackal_rot = rospy.ServiceProxy('get_jackal_rot', JackalRot)
		print("get_jackal_rot service ready.")
		#############################################################################


		self.at_flag = False  # bool for rover is at flag
		self.emergency_stop = False  # emergency stop signal (from rf controller)

		# Actuator settings:
		self.actuator_min = -25  # accounting for scale factor on arduino (65 - 90) + 1 !!TEST THIS ONE!!
		self.actuator_max = 47  # accounting for scale factor on arduino (138 - 90) - 1
		self.actuator_home = 0
		self.actuator_stop = 0

		# Throttle settings (updated 07/05/18):
		self.throttle_home = 120  # idle state
		self.throttle_min = 120  # lowest throttle state
		self.throttle_max = 60  # full throttle!

		# Articulation settings:
		self.turn_left = 0  # publish this value to turn left
		self.turn_right = 2  # publish this value to turn right
		self.max_angle = 22  # max right (relative to driver/rover)
		self.min_angle = -22  # max left (relative to driver/rover)
		self.min_angle_tolerance = 1.0  # min allowable angle tolerance

		print("Red Rover nav controller ready.")

		rospy.spin()



	# def flag_callback(self, flag_msg):
	# 	"""
	# 	Subscribes to /at_flag topic that's being published by
	# 	jackal_flag_node.py. Needs to stop Jackal if at_flag is True
	# 	"""
	# 	if flag_msg.data == True or flag_msg == True:
	# 		print("Stopping cause we're at the flag!!!")
	# 		self.at_flag = True  # sets main at_flag to True for robot..
	# 	else:
	# 		self.at_flag = False



	# def rf_stop_callback(self, stop_msg):
	# 	"""
	# 	/rf_stop is an emergency stop from the arduino, which uses a 
	# 	32197-MI 4 Ch. remote control receiver
	# 	"""
	# 	# print("Received RF stop message! {}".format(stop_msg))
	# 	if stop_msg.data == True:
	# 		print("Received RF stop message! {}".format(stop_msg))
	# 		self.emergency_stop = True
	# 	else:
	# 		self.emergency_stop = False



	# def drive_forward(self, goal_distance, look_ahead):

	# 	# TODO: set min allowable drive distance..

	# 	_drive_val = None
	# 	if goal_distance > 0:
	# 		print("Driving fowards {} meters..".format(goal_distance))
	# 		_drive_val = self.actuator_home  # TODO: change to move forward
	# 	elif goal_distance < 0:
	# 		print("Driving backwards {} meters..".format(goal_distance))
	# 		_drive_val = self.actuator_home  # TODO: change to move backward
	# 	else:
	# 		print("Drive distance is zero, canceling drive request..")
	# 		return

	# 	self.actuator_pub.publish(_drive_val)  # initiate movement

	# 	distance = 0  # distance traveled
	# 	curr_pose_utm = self.get_current_position()  # current position in utm
	# 	x_start, y_start = curr_pose_utm[0], curr_pose_utm[1]  # sets starting x,y position for drive

	# 	while distance < (goal_distance - look_ahead) and not self.at_flag and not self.emergency_stop and not rospy.is_shutdown():
	# 		rospy.sleep(1.0/rate)
	# 		curr_pose_utm = self.get_current_position()
	# 		distance = sqrt(pow((curr_pose_utm[0] - x_start), 2) + 
	# 						pow((curr_pose_utm[1] - y_start), 2))

		# # Stops rover and takes sample if at a flag:
		# if self.at_flag:
		# 	print("Near flag! Stopping Red Rover for collecting a sample..")
		# 	self.stop_driving()  # stopping rover
		# 	print("Pausing briefly before making request to collect a sample..")
		# 	rospy.sleep(5)
		# 	print("Calling sample collector service to initiate data collection while robot is stopped..")
		# 	sample_collector_result = self.start_sample_collection('collect')
		# 	print("Sample collected: {}".format(sample_collector_result))
		# 	print("Pausing briefly before continuing on to next flag..")
		# 	rospy.sleep(5)
		# 	self.at_flag = False

		# 	# TODO: Will probably need to rev engine up for sample collection..

		# return



	def translate_angle_with_imu(self, goal_angle):
		"""
		Uses IMU to translate a number of degrees (goal_angle), but stops
		if it exceeds the turning boundaries of the red rover, which uses
		the pivot data to determine.
		"""
		_turn_val = self.no_turn_val  # initializes turn to not turn

		print("Angle to translate: {}".format(goal_angle))

		if goal_angle > 0:
			print("Turning right..")
			_turn_val = self.turn_right_val  # value to turn right
		elif goal_angle < 0:
			print("Turning left..")
			_turn_val = self.turn_left_val  # value to turn left

		turn_angle = 0
		last_angle = self.get_jackal_rot().jackal_rot  # get angle from IMU (in radians)

		# while abs(turn_angle) < abs(goal_angle) and not self.at_flag and not rospy.is_shutdown():
		while abs(turn_angle) < abs(radians(goal_angle)) and not rospy.is_shutdown():

			# self.cmd_vel.publish(move_cmd)

			# print("Current angle: {}, Current pivot: {}".format(self.last_angle, self.current_pivot))

			self.articulator_pub.publish(_turn_val)

			rospy.sleep(1.0/self.rate)

			curr_angle = self.get_jackal_rot().jackal_rot
			delta_angle = self.normalize_angle(curr_angle - last_angle)
			turn_angle += delta_angle
			last_angle = curr_angle

			if delta_angle == 0.0:
				print("Delta angle is 0, breaking out of turning loop..")
				break

		self.articulator_pub.publish(self.no_turn_val)  # stop turning once goal angle is reached.

		# if self.emergency_stop:
		# 	print("Emergency stop from RF remote received, stopping turning routine..")

		return



	def get_current_position(self):
		"""
		Calls jackal_pos_server and converts the NavSatFix type
		into UTM format.
		"""
		curr_pose = self.get_jackal_pos()
		curr_pose_utm = utm.from_latlon(curr_pose.jackal_fix.latitude, curr_pose.jackal_fix.longitude)
		return curr_pose_utm



	def normalize_angle(self, angle):
		res = angle
		while res > pi:
			res -= 2.0 * pi
		while res < -pi:
			res += 2.0 * pi
		return res



	def stop_driving(self):
		"""
		Stops red rover from driving. Sets throttle to home state,
		sets drive actuator to neutral position, and keeps
		pivot where it is.
		"""
		print("Stopping Red Rover..")
		self.actuator_pub.publish(self.actuator_stop)  # stops driving
		self.throttle_pub.publish(self.throttle_home)  # idles engine
		print("Red Rover stopped.")
		return



	def shutdown_all(self):
		print("Shutting down rover, stopping drive, lowering throttle rpms..")
		self.actuator_pub.publish(self.actuator_stop)
		rospy.sleep(1)
		self.throttle_pub.publish(120)
		rospy.sleep(1)
		print("Red rover stopped.")







if __name__ == '__main__':

	try:
		NavController()
	except rospy.ROSInterruptException:
		raise