#!/usr/bin/env python

import roslib
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float64
from simple_navigation_goals.srv import *
import tf
from math import radians, copysign, sqrt, pow, pi, degrees
import PyKDL
import utm



# Global settings:
tf_listener = tf.TransformListener()
odom_frame = '/odom'
base_frame = '/base_link'
linear_speed = 0.3  # units of m/s
rate = 20  # Hz
angular_speed = 0.3
angular_tolerance = rospy.get_param("~angular_tolerance", radians(2)) # degrees to radians




class NavController:

	def __init__(self):

		rospy.Subscriber("/at_flag", Bool, self.flag_callback)  # sub to /at_flag topic from jackal_flags_node.py
		# rospy.Subscriber("sample_collected", Bool, self.sample_collected_callback)
		rospy.Subscriber("/rf_stop", Bool, self.rf_stop_callback, queue_size=1)




		# rospy.Subscriber('/stop_rover', Bool, self.stop_rover_callback)
		# rospy.Subscriber('/start_rover', Bool, self.start_rover_callback)




		self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)  # see http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers#Choosing_a_good_queue_size

		self.move_cmd = Twist()

		# self.sample_publisher = rospy.Publisher('collect_sample', Bool, queue_size=1)

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

		self.at_flag = False
		self.emergency_stop = False



	def flag_callback(self, flag_msg):
		"""
		Subscribes to /at_flag topic that's being published by
		jackal_flag_node.py. Needs to stop Jackal if at_flag is True
		"""
		if flag_msg.data == True or flag_msg == True:
			print("Stopping cause we're at the flag!!!")
			self.at_flag = True  # sets main at_flag to True for robot..
		else:
			self.at_flag = False



	def rf_stop_callback(self, stop_msg):
		"""
		/rf_stop is an emergency stop from the arduino, which uses a 
		32197-MI 4 Ch. remote control receiver
		"""
		if stop_msg.data == True:
			print("Received RF stop message! {}".format(stop_msg))
			self.emergency_stop = True
		else:
			self.emergency_stop = False



	def drive_forward(self, goal_distance, look_ahead):

		print("Driving Jackal {} meters..".format(goal_distance))

		move_cmd = Twist()
		move_cmd.linear.x = linear_speed

		curr_pose_utm = self.get_current_position()  # returns UTM tuple

		distance = 0
		x_start, y_start = curr_pose_utm[0], curr_pose_utm[1]

		while distance < (goal_distance - look_ahead) and not self.at_flag and not self.emergency_stop and not rospy.is_shutdown():

			# while self.emergency_stop:
			# 	print("Emergency stop message received, stopping rover..")
			# 	self.cmd_vel.publish(Twist())
			# 	print("Rover stopped, maybe.")

			rospy.sleep(1.0/rate)

			curr_pose_utm = self.get_current_position()

			distance = sqrt(pow((curr_pose_utm[0] - x_start), 2) + 
							pow((curr_pose_utm[1] - y_start), 2))

			self.cmd_vel.publish(move_cmd)


		# Check to see if robot is at flag
		if self.at_flag:
			print("Calling sample collector service to initiate data collection while robot is stopped..")
			sample_collector_result = self.start_sample_collection('collect')
			print("Sample collected: {}".format(sample_collector_result))
			self.at_flag = False

		elif self.emergency_stop:
			print("Emergency stop from RF device triggered break in driving routine..")

		return



	def execute_turn(self, goal_angle):
		"""
		Function for executing a turn in the odom frame.
		"""
		move_cmd = Twist()

		if goal_angle > 0:
			move_cmd.angular.z = angular_speed

		if goal_angle < 0:
			move_cmd.angular.z = -angular_speed

		turn_angle = 0
		last_angle = self.get_jackal_rot().jackal_rot  # get angle from IMU (in radians)

		# while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not self.at_flag  and not self.emergency_stop and not rospy.is_shutdown():
		while abs(turn_angle) < abs(goal_angle) and not self.at_flag  and not self.emergency_stop and not rospy.is_shutdown():

			self.cmd_vel.publish(move_cmd)
			rospy.sleep(1.0/rate)

			curr_angle = self.get_jackal_rot().jackal_rot
			delta_angle = self.normalize_angle(curr_angle - last_angle)
			turn_angle += delta_angle
			last_angle = curr_angle

			if delta_angle == 0.0:
				print("Delta angle is 0, breaking out of turning loop..")
				break

		if self.emergency_stop:
			print("Emergency stop from RF remote received, stopping turning routine..")

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







if __name__ == '__main__':
	rospy.init_node('nav_controller')
	nc = NavController()
	rospy.spin()