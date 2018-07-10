#!/usr/bin/env python

import roslib
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from std_msgs.msg import Bool, String, Float64, UInt8
from simple_navigation_goals.srv import *
from math import radians, copysign, sqrt, pow, pi, degrees
import PyKDL
import utm
import json



# Global settings:
rate = 10  # Turning and driving loop rate for red rover
angular_tolerance = rospy.get_param("~angular_tolerance", radians(2)) # degrees to radians




class NavController(object):

	def __init__(self):

		

		# Subscribers:
		# rospy.Subscriber("/at_flag", Bool, self.flag_callback)  # sub to /at_flag topic from jackal_flags_node.py
		# rospy.Subscriber("/rf_stop", Bool, self.rf_stop_callback, queue_size=1)
		rospy.Subscriber("/driver/encoder_velocity", Float64, self.rover_velocity_callback)
		rospy.Subscriber("/driver/pivot", Float64, self.rover_pivot_callback, queue_size=1)
		rospy.Subscriber("/driver/test/set_throttle", Float64, self.set_throttle_callback, queue_size=1)
		rospy.Subscriber("/driver/test/execute_turn", Float64, self.articulator_turn_callback)

		# Publishers:
		self.actuator_pub = rospy.Publisher('/driver/linear_drive_actuator', Float64, queue_size=1)  # TODO: double check queue sizes..
		self.throttle_pub = rospy.Publisher('/driver/throttle', UInt8, queue_size=1)  # TODO: double check queue sizes..
		self.articulator_pub = rospy.Publisher('/driver/articulation_relay', Float64, queue_size=1)  # TODO: double check queue sizes..
		# self.sample_publisher = rospy.Publisher('collect_sample', Bool, queue_size=1)


		# Establishing ROS Service connections:
		#############################################################################
		
		# TEMPORARILY REMOVED FOR TESTING red_rover_drive_2.py!!!

		# print("Waiting for start_sample_collection service..")
		# rospy.wait_for_service('start_sample_collection')
		# self.start_sample_collection = rospy.ServiceProxy('start_sample_collection', SampleCollection)
		# print("start_sample_collection service ready.")

		# print("Waiting for get_jackal_pos service..")
		# rospy.wait_for_service('get_jackal_pos')
		# self.get_jackal_pos = rospy.ServiceProxy('get_jackal_pos', JackalPos)
		# print("get_jackal_pos service ready.")

		# print("Waiting for get_jackal_rot service..")
		# rospy.wait_for_service('get_jackal_rot')
		# self.get_jackal_rot = rospy.ServiceProxy('get_jackal_rot', JackalRot)
		# print("get_jackal_rot service ready.")
		#############################################################################


		self.at_flag = False  # bool for rover is at flag
		self.emergency_stop = False  # emergency stop signal (from rf controller)

		# Actuator settings:
		self.actuator_min = -25  # accounting for scale factor on arduino (65 - 90) + 1 !!TEST THIS ONE!!
		self.actuator_max = 47  # accounting for scale factor on arduino (138 - 90) - 1
		self.actuator_home = 0
		self.actuator_stop = 0
		self.actuator_drive_slow = 20  # NOTE: TEST THIS TO MAKE SURE IT'S "SLOW"

		# Throttle settings (updated 07/05/18):
		self.throttle_home = 120  # idle state
		self.throttle_min = 120  # lowest throttle state
		self.throttle_max = 60  # full throttle!
		self.throttle_drive_slow = 100  # throttle setting for slow driving??	

		# Articulation settings:
		self.turn_left_val = 0  # publish this value to turn left
		self.turn_right_val = 2  # publish this value to turn right
		self.no_turn_val = 1  # publish this value to not turn??????
		self.max_pivot = 22  # max right (relative to driver/rover)
		self.min_pivot = -22  # max left (relative to driver/rover)
		self.min_pivot_tolerance = 1.0  # min allowable angle tolerance

		self.rate = 10  # loop rate for turning and such (Hz)



	def rover_velocity_callback(self, msg):
		"""
		Subscriber callback for big rover's velocity.
		"""
		# print("Rover velocity callback message: {}".format(msg))
		pass
		


	def rover_pivot_callback(self, msg):
		"""
		Subscriber callback for the big rover's current angle/pivot.
		"""
		# print("Rover pivot callback message: {}".format(msg))
		pass



	def set_throttle_callback(self, msg):
		"""
		Subscriber callback for running throttle test.
		"""
		
		throttle_val = msg.data

		print("Setting throttle to {}".format(throttle_val))
		print("Publishing single value, {}, to /driver/throttle topic".format(self.throttle_low))

		if throttle_val < self.throttle_high or throttle_val > self.throttle_low:
			print("!!! Must provide a throttle value between {} (full throttle) and {} (low throttle) !!!".format(self.throttle_high, self.throttle_low))
			return

		if not throttle_val:
			throttle_val = self.throttle_home

		self.throttle_pub.publish(throttle_val)



	def articulator_turn_callback(self, msg):
		"""
		Subscriber callback to initate a turn for testing
		the red rover's articulation.
		"""
		print("Received message on articulator_turn_callback to turn {} degrees..".format(msg.data))
		self.turn_to_pivot(msg.data)

		

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



	def check_pivot_bounds(self, goal_pivot):
		"""
		Trims goal pivot based on min/max allowable angles:
		"""
		if goal_pivot > self.max_pivot:
			print("Requested rover pivot is too large, setting to max allowable: {}".format(self.max_pivot))
			goal_pivot = self.max_pivot
		elif goal_pivot < self.min_pivot:
			print("Request rover pivot is too small, setting to min allowable: {}".format(self.min_pivot))
			goal_pivot = self.min_pivot
		return goal_pivot



	def turn_left(self, goal_pivot):
		"""
		Turn left loop.
		"""
		while self.current_pivot > goal_pivot and not rospy.is_shutdown():
			print("Rover pivot: {}".format(self.current_pivot))
			rospy.sleep(0.1)  # delay 100ms
			self.articulator_pub.publish(self.turn_left_val)  # turn left

		self.articulator_pub.publish(self.no_turn_val)

		return



	def turn_right(self, goal_pivot):
		"""
		Turn right loop.
		"""
		while self.current_pivot < goal_pivot and not rospy.is_shutdown():
			print("Rover pivot: {}".format(self.current_pivot))
			rospy.sleep(0.1)  # delay 100ms
			self.articulator_pub.publish(self.turn_right_val)  # turn right

		self.articulator_pub.publish(self.no_turn_val)

		return




	def turn_to_pivot(self, goal_pivot):
		"""
		Executes a turn to a specific angle using the
		/driver/pivot topic.
		Inputs:
			goal_pivot - angle to turn to based on rover's /driver/pivot topic
		"""

		goal_pivot = self.check_pivot_bounds(goal_pivot)

		turn_angle = goal_pivot - self.current_pivot  # determines direction to turn
		print("Turning {} degrees..".format(turn_angle))

		rospy.sleep(1)

		if turn_angle < -self.min_pivot_tolerance:
			self.turn_left(goal_pivot)  # start turning left
		elif turn_angle > self.min_pivot_tolerance:
			self.turn_right(goal_pivot)  # start turning right
		else:
			print("Turn angle is zero, canceling turn request..")
			return  # don't turn if angle is 0



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
		while abs(turn_angle) < abs(radians(goal_angle)) and not self.at_flag and not rospy.is_shutdown():

			# self.cmd_vel.publish(move_cmd)

			# print("Current angle: {}, Current pivot: {}".format(self.last_angle, self.current_pivot))

			self.articulator_pub.publish(_turn_val)

			rospy.sleep(1.0/self.rate)

			curr_angle = self.get_jackal_rot().jackal_rot
			delta_angle = self.normalize_angle(curr_angle - last_angle)
			turn_angle += delta_angle
			last_angle = curr_angle

			if delta_angle == 0.0:
				# print("Delta angle is 0, breaking out of turning loop..")
				break

		self.articulator_pub.publish(self.no_turn_val)  # stop turning once goal angle is reached.

		# if self.emergency_stop:
		# 	print("Emergency stop from RF remote received, stopping turning routine..")

		return



	# def get_current_position(self):
	# 	"""
	# 	Calls jackal_pos_server and converts the NavSatFix type
	# 	into UTM format.
	# 	"""
	# 	curr_pose = self.get_jackal_pos()
	# 	curr_pose_utm = utm.from_latlon(curr_pose.jackal_fix.latitude, curr_pose.jackal_fix.longitude)
	# 	return curr_pose_utm



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
		print("Shutting down rover: stopping drive, lowering throttle rpms..")
		self.actuator_pub.publish(self.actuator_stop)
		rospy.sleep(1)
		self.throttle_pub.publish(self.throttle_home)
		rospy.sleep(1)
		self.articulator_pub.publish(self.no_turn_val)
		rospy.sleep(1)
		print("Red rover stopped.")







if __name__ == '__main__':

	try:
		rospy.init_node('red_rover_nav_controller')
		nc = NavController()
		print("Red Rover nav controller ready.")
		rospy.spin()
	except rospy.ROSInterruptException:
		raise