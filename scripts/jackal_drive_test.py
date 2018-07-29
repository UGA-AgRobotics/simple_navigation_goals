#!/usr/bin/env python

"""
Testing Jackal row following, with dubins at end of row
to get down another row. Eventually will incorporate a 
boundary box as well, for the safety.
"""

import roslib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64, UInt8, Int64
from sensor_msgs.msg import NavSatFix
from mico_leaf_msgs.srv import start_sample
import sys
import math
import json
import utm
from math import radians, copysign, sqrt, pow, pi, degrees
import PyKDL
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import dubins

# Local package requirements:
from nav_tracks import NavTracks
from nav_nudge import NavNudge
import orientation_transforms



class SingleGoalNav(object):
	"""
	Testing Rover navigation.
	Drives based on rover's position, a look-ahead goal in a course,
	and its orientatiion. Subscribes to GPS and IMU topics.
	"""

	def __init__(self, path_json, nudge_factor=None):
		
		# Give the node a name
		rospy.init_node('single_goal_nav')

		# Subscribers:
		rospy.Subscriber("/start_driving", Bool, self.start_driving_callback, queue_size=1)
		rospy.Subscriber("/fix", NavSatFix, self.rover_position_callback, queue_size=1)
		rospy.Subscriber('/imu/data', Imu, self.rover_imu_callback, queue_size=1)  # NOTE: TEMP TESTING WITH JACKAL'S IMU!!!!!
		rospy.Subscriber("/at_flag", Bool, self.flag_callback, queue_size=1)  # sub to /at_flag topic from jackal_flags_node.py
		rospy.Subscriber("/flag_index", Int64, self.flag_index_callback, queue_size=1)
		rospy.Subscriber("/stop_gps", Bool, self.stop_gps_callback, queue_size=1)

		# Publisher for controller jackal:
		self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)  # see http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers#Choosing_a_good_queue_size
		
		self.move_cmd = Twist()  # Data type to move jackal
		self.linear_speed = 0.3  # jackal's linear speed
		self.angular_speed = 0.1  # jackal's angular speed

		# Set rospy to exectute a shutdown function when terminating the script
		rospy.on_shutdown(self.shutdown)

		# How fast will we check the odometry values?
		self.rate = 10
		
		# Set the equivalent ROS rate variable
		self.r = rospy.Rate(self.rate)

		self.path_json = path_json  # The path/course the red rover will follow!

		if nudge_factor and isinstance(nudge_factor, float):
			print("Using nudge factor of {} to shift the course!".format(nudge_factor))
			nn = NavNudge(json.dumps(path_json), nudge_factor, 0.2)  # NOTE: HARD-CODED SPACING FACTOR TO 0.2M FOR NOW
			self.path_json = nn.nudged_course


		self.path_array = None  # path converted to list of [easting, northing]

		self.look_ahead = 1.5  # look-ahead for target index, in meters

		self.angle_tolerance = 0.1  # angle tolerance in degrees

		self.angle_trim = 2.0  # max angle inc per iteration (in degrees)

		self.target_index = 0  # index in course that's the goal position
		
		self.current_goal = None  # [easting, northing] array
		self.current_pos = None  # [easting, northing] array
		self.current_angle = None  # angle from imu in radians

		self.np_course = None  # lazy np array version of course for certain manipulations

		self.at_flag = False
		self.flag_index = None

		self.stop_gps = False

		print("Jackal driver ready.")



	def stop_gps_callback(self, msg):
		"""
		Subs to /stop_gps topic from emlid_socketio_client node.
		Sends a True if rover loses a fix.
		"""
		if msg.data == True:
			print("Received True on /stop_gps, rover has lost a fix..")
			self.stop_gps = True
		else:
			self.stop_gps = False



	def flag_index_callback(self, msg):
		"""
		Keeps track of flag index from the flag node.
		Sends this integer to the sample collector.
		"""
		# print("Setting flag index to {}".format(msg.data))
		self.flag_index = msg.data




	def flag_callback(self, flag_msg):
		"""
		Subscribes to /at_flag topic that's being published by
		jackal_flag_node.py. Needs to stop Jackal if at_flag is True
		"""
		if flag_msg.data == True:
			print("Stopping cause we're at the flag!!!")
			self.at_flag = True  # sets main at_flag to True for robot..
		else:
			self.at_flag = False



	def start_driving_callback(self, msg):
		"""
		Initiates driving routine.
		The course file that was referenced when initiating the RedRoverDrive class
		is converted to a list of [easting, northing] pairs, then initiate the rover
		to drive and follow the course.
		"""
		if msg.data == True:

			if not self.path_json:
				print("Waiting for drive node to be started..")
				return

			if not isinstance(self.path_json, list):
				nt = NavTracks()
				path_array = nt.get_track_from_course(self.path_json)  # builds list of [easting, northing] pairs from course file
			else:
				path_array = self.path_json  # assuming it's already a list of [easting, northing] pairs..

			print("The Course: {}".format(path_array))
			print("Starting path following routine..")

			self.target_index = 0

			self.start_path_following(path_array, self.target_index)



	def rover_position_callback(self, msg):
		"""
		Position from GPS converted to UTM.

		It also checks the rover's distance between its current
		position and the goal position in the course. Moves to
		next goal in course if rover is within look ahead distance.

		NOTE: Does the target index need to increment here and/or turn loop?

		"""		
		_lat, _lon = msg.latitude, msg.longitude
		curr_pose_utm = utm.from_latlon(_lat, _lon)
		self.current_pos = [curr_pose_utm[0], curr_pose_utm[1]]



	def rover_imu_callback(self, msg):
		"""
		Angle from IMU in radians.
		"""
		self.current_angle = self.quat_to_angle(msg.orientation)



	def call_micoleaf_service(self, flag_ind):

		# Mico Leaf Service:
		print("Waiting for /mico_leaf1/sample_service..")
		rospy.wait_for_service('/mico_leaf1/sample_service')
		self.start_sample_collection = rospy.ServiceProxy('/mico_leaf1/sample_service', start_sample)
		print("start_sample_collection service ready.")

		rospy.sleep(2)

		print("Calling arm service to collect samples.")

		try:
			test_val = self.start_sample_collection(flag_ind)
			print("val returned: {}".format(test_val.end_sample))
		except rospy.ServiceException as e:
			print("an exception happend.")
			print("exception: {}".format(e))

		print("Samples completed!")

		return



	def quat_to_angle(self, quat):
		"""
		Converts quaternion to angle.
		"""
		rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
		return rot.GetRPY()[2]



	def start_path_following(self, path_array, init_target):

		if not isinstance(path_array, list):
			self.shutdown()
			raise Exception("Path must be a list of [easting, northing] pairs..")

		if len(path_array) < 1:
			self.shutdown()
			raise Exception("Path must be at least one point..")

		i = 0
		while not self.current_pos:
			print("({}s) Waiting for GPS data from /fix topic..")
			rospy.sleep(1)
			i += 1

		if self.stop_gps:
			self.wait_for_fix()
			

		print("INITIAL TARGET: {}".format(init_target))

		self.np_course = np.array(path_array)  # sets numpy array of course\

		rospy.sleep(2)  # give messages time to publish

		_curr_utm = self.current_pos  # gets current /fix
		self.target_index = self.calc_target_index(_curr_utm, init_target, self.np_course[:,0], self.np_course[:,1])  # try using int_target
		self.current_goal = path_array[self.target_index]  # sets current goal


		print("Total length of path array: {}".format(len(path_array)))
		print("Initial target index: {}".format(self.target_index))
		print("Initial target UTM: {}".format(self.current_goal))


		# Sleep routine for testing:
		print("Pausing 10 seconds before initiating driving (to have time to run out there)...")
		rospy.sleep(10)

		
		print("Starting driving routine.")

		move_cmd = Twist()
		move_cmd.linear.x = self.linear_speed

		self.cmd_vel.publish(move_cmd)  # start driving straight!



		# ###################################################################
		# # This loop calculates a turn angle a look-ahead distance away,
		# # then begins to execute the turn.
		# ###################################################################
		# while not rospy.is_shutdown():

		# 	if self.at_flag:
		# 		print("At a flag in the course! Stopping the rover to take a sample.")
		# 		self.execute_flag_routine()

		# 	if self.stop_gps:
		# 		print("Lost GPS fix.. Stopping the rover until fix is obtained..")
		# 		self.wait_for_fix()

		# 	rospy.sleep(0.2)

		# 	_curr_utm = self.current_pos  # gets current utm
		# 	self.target_index = self.calc_target_index(_curr_utm, self.target_index, self.np_course[:,0], self.np_course[:,1])

		# 	print("target index: {}".format(self.target_index))

		# 	if self.target_index == None:
		# 		print("Assuming end of course is reached! Stopping rover.")
		# 		self.shutdown()
		# 		return

		# 	self.current_goal = self.np_course.tolist()[self.target_index]
		# 	_curr_angle = self.current_angle  # gets current angle in radians

		# 	A = (_curr_utm[0], _curr_utm[1], _curr_angle)
		# 	B = (self.current_goal[0], self.current_goal[1], 0)  # note: B angle not used..

		# 	turn_angle = orientation_transforms.initiate_angle_transform(A, B)  # note: flipped sign of turn from imu

		# 	print("Initial turn angle: {}".format(turn_angle))

		# 	if abs(turn_angle) > abs(self.angle_tolerance):

		# 		if turn_angle < -self.angle_trim:
		# 			turn_angle = -self.angle_trim

		# 		elif turn_angle > self.angle_trim:
		# 			turn_angle = self.angle_trim

		# 		print("Telling Rover to turn {} degreess..".format(turn_angle))

		# 		self.translate_angle_with_imu(turn_angle)

		# 		print("Finished turn.")


		# print("Finished driving course..")
		# print("Shutting down Jackal..")
		# self.shutdown()



	def execute_row_follow(self):

		###################################################################
		# This loop calculates a turn angle a look-ahead distance away,
		# then begins to execute the turn.
		###################################################################
		while not rospy.is_shutdown():

			rospy.sleep(0.2)

			if self.at_flag:
				print("At a flag in the course! Stopping the rover to take a sample.")
				self.execute_flag_routine()

			if self.stop_gps:
				print("Lost GPS fix.. Stopping the rover until fix is obtained..")
				self.wait_for_fix()

			if self.target_index == None:
				print("Assuming end of course is reached!")
				# self.shutdown()


				# TODO: using multi-row course file, run dubins, where A = current position,
				# and B = the next row (have to use angle to determine direction to go).
				

				return

			_curr_utm = self.current_pos  # gets current utm
			self.target_index = self.calc_target_index(_curr_utm, self.target_index, self.np_course[:,0], self.np_course[:,1])

			print("target index: {}".format(self.target_index))

			self.current_goal = self.np_course.tolist()[self.target_index]
			_curr_angle = self.current_angle  # gets current angle in radians

			A = (_curr_utm[0], _curr_utm[1], _curr_angle)
			B = (self.current_goal[0], self.current_goal[1], 0)  # note: B angle not used..

			turn_angle = orientation_transforms.initiate_angle_transform(A, B)  # note: flipped sign of turn from imu

			print("Initial turn angle: {}".format(turn_angle))

			if abs(turn_angle) > abs(self.angle_tolerance):

				if turn_angle < -self.angle_trim:
					turn_angle = -self.angle_trim

				elif turn_angle > self.angle_trim:
					turn_angle = self.angle_trim

				print("Telling Rover to turn {} degreess..".format(turn_angle))

				self.translate_angle_with_imu(turn_angle)

				print("Finished turn.")


		print("Finished driving course..")
		print("Shutting down Jackal..")
		self.shutdown()




	def calc_target_index(self, current_position, current_goal_index, cx, cy):
		"""
		From red_rover_model pure_puruit module. Loops through course
		points (x and y) and builds a list of the diff b/w robot's position and
		each x and y in t{he course. Finally, 
		"""
		# note: numpy seems to return blank array if out of index, so
		# it should return None at end of course.

		dx = [current_position[0] - icx for icx in cx]  # diff b/w robot's position and all x values in course (starting at current goal, onward)
		dy = [current_position[1] - icy for icy in cy]  # diff b/w robot's position and all y values in course (starting at current goal, onward)

		d = [math.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]  # scalar diff b/w robot and course values

		print("Determining goal point based on look-ahead of {}".format(self.look_ahead))

		ind = d.index(min(d))  # index of closest goal to robot

		print("Min index: {}".format(ind))

		# loops list, starting at closest point to robot:
		for _diff in d[ind:]:
			if _diff > self.look_ahead:
				return ind				
			ind += 1

		return None



	def wait_for_fix(self):
		"""
		Hangs while until it receives a True on /stop_gps topic.
		"""
		i = 0
		while self.stop_gps:
			print("({}s) Waiting for GPS to obtain a fix..".format(i))
			rospy.sleep(1.0)
			i += 1

		self.stop_gps = False
		return




	def execute_flag_routine(self):
		"""
		Routine to run when the rover is at a flag.
		"""
		print("Making sure rover is stopped, then making request to take a sample..")
		rospy.sleep(0.1)
		self.cmd_vel.publish(Twist())

		# Call sample collector service here..
		########################################################################
		print("Pausing 10s to simulate a sample collection routine..")
		rospy.sleep(10)

		# print("Pausing 5s, then calling mico leaf service..")
		# rospy.sleep(5)
		# self.throttle_pub.publish(self.throttle_max)
		# rospy.sleep(1)

		# print("Calling mico_leaf1 service, bin {}".format(self.flag_index))
		# self.call_micoleaf_service(self.flag_index)
		# print("mico_leaf1 service complete.")

		# rospy.sleep(1)
		# self.throttle_pub.publish(self.throttle_drive_slow)
		# rospy.sleep(1)
		########################################################################

		_curr_utm = self.current_pos
		self.target_index = self.calc_target_index(_curr_utm, self.target_index, self.np_course[:,0], self.np_course[:,1])

		updated_path =self.np_course.tolist()[self.target_index:]  # set remaining path to follow
		self.np_course = np.array(updated_path)  # updates np array of course

		self.at_flag = False  # set at_flag to False after sample is collected..

		return



	def translate_angle_with_imu(self, goal_angle):
		"""
		Uses IMU to translate a number of degrees (goal_angle), but stops
		if it exceeds the turning boundaries of the red rover, which uses
		the pivot data to determine.
		"""

		# Below move_cmd sequence is an attempt to go forward and turn at the same time with the jackal!!!
		move_cmd = Twist()

		if goal_angle > 0:
			move_cmd.angular.z = self.angular_speed
		elif goal_angle < 0:
			move_cmd.angular.z = -self.angular_speed

		move_cmd.linear.x = self.linear_speed


		turn_angle = 0
		last_angle = self.current_angle

		while abs(turn_angle) < abs(radians(goal_angle)) and not self.at_flag and not rospy.is_shutdown():

			self.cmd_vel.publish(move_cmd)

			rospy.sleep(1.0/self.rate)

			curr_angle = self.current_angle
			delta_angle = self.normalize_angle(curr_angle - last_angle)
			turn_angle += delta_angle
			last_angle = curr_angle

			if delta_angle == 0.0:
				break

		return


	def normalize_angle(self, angle):
		res = angle
		while res > pi:
			res -= 2.0 * pi
		while res < -pi:
			res += 2.0 * pi
		return res



	def determine_drive_distance(self, A, B):
		return math.sqrt((B[1] - A[1])**2 + (B[0] - A[0])**2)




	def shutdown(self):
		"""
		Always stop the robot when shutting down the node
		"""
		print("Stopping the Jackal..")
		self.cmd_vel.publish(Twist())







if __name__ == '__main__':

	try:
		course_filename = sys.argv[1]
	except IndexError:
		raise IndexError("Course not specified. Add course filename as arg when running basic_drive_5.py")

	try:
		nudge_factor = float(sys.argv[2])
	except Exception:
		print("No nudge factor provided, assuming 0..")
		nudge_factor = None

	coursefile = open(course_filename, 'r')
	course = json.loads(coursefile.read())

	print("Course to follow: {}".format(course_filename))

	try:
		SingleGoalNav(course, nudge_factor)
	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation terminated.")
		rospy.loginfo("Shutting down drive node!")
		raise Exception("basic drive ROS node exception")

	rospy.spin()