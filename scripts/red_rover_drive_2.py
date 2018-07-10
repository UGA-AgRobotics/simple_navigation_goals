#!/usr/bin/env python

import roslib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64, UInt8
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

# Local package requirements:
from nav_tracks import NavTracks
from red_rover_nav_controller import NavController
import orientation_transforms



# nc = NavController()



class SingleGoalNav(object):
	"""
	Testing Rover navigation.
	Drives based on rover's position, a look-ahead goal in a course,
	and its orientatiion. Subscribes to GPS and IMU topics.
	"""

	def __init__(self, path_json=None):
		
		# Give the node a name
		rospy.init_node('single_goal_nav')

		# Subscribers:
		rospy.Subscriber("/start_driving", Bool, self.start_driving_callback, queue_size=1)
		rospy.Subscriber("/fix", NavSatFix, self.rover_position_callback, queue_size=1)
		rospy.Subscriber('/phidget/imu/data', Imu, self.rover_imu_callback, queue_size=1)
		rospy.Subscriber("/at_flag", Bool, self.flag_callback)  # sub to /at_flag topic from jackal_flags_node.py

		# Publishers:
		self.actuator_pub = rospy.Publisher('/driver/linear_drive_actuator', Float64, queue_size=1)  # TODO: double check queue sizes..
		self.throttle_pub = rospy.Publisher('/driver/throttle', UInt8, queue_size=1)  # TODO: double check queue sizes..
		self.articulator_pub = rospy.Publisher('/driver/articulation_relay', Float64, queue_size=1)  # TODO: double check queue sizes..



		print("Waiting for /mico_leaf1/sample_service..")
		rospy.wait_for_service('/mico_leaf1/sample_service')
		# self.start_sample_collection = rospy.ServiceProxy('start_sample_collection', SampleCollection)
		self.start_sample_collection = rospy.ServiceProxy('/mico_leaf1/sample_service', start_sample)
		print("start_sample_collection service ready.")


		try:
			test_val = self.start_sample_collection(2)
			print("val returned: {}".format(test_val.end_sample))
		except rospy.ServiceException as e:
			print("an exception happend.")
			print("exception: {}".format(e))


		return




		# Set rospy to exectute a shutdown function when terminating the script
		rospy.on_shutdown(self.shutdown)

		# How fast will we check the odometry values?
		self.rate = 10
		
		# Set the equivalent ROS rate variable
		self.r = rospy.Rate(self.rate)

		self.angle_tolerance = 0.1  # angle tolerance in degrees


		# print("setting path json..")

		self.path_json = path_json  # The path/course the red rover will follow!


		# print("path json set: {}".format(self.path_json))


		self.path_array = None  # path converted to list of [easting, northing]

		self.look_ahead = 1.5  # meters

		self.angle_trim = 3.0  # max angle inc per iteration (in degrees)

		# Articulation settings:
		self.turn_left_val = 0  # publish this value to turn left
		self.turn_right_val = 2  # publish this value to turn right
		self.no_turn_val = 1  # publish this value to not turn??????

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

		self.target_index = None  # index in course that's the goal position
		self.current_goal = None  # [easting, northing] array
		self.current_pos = None  # [easting, northing] array
		self.current_angle = None  # angle from imu in radians

		self.np_course = None  # lazy np array version of course for certain manipulations

		self.at_flag = False  # todo: subscribe to at_flag topic?



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

			# Gets track to follow:
			nt = NavTracks()
			path_array = nt.get_track_from_course(self.path_json)  # builds list of [easting, northing] pairs from course file
			
			self.path_array = path_array

			print("The Course: {}".format(path_array))
			print("Starting path following routine..")

			print("Setting throttle and drive actuator to home states..")
			self.throttle_pub.publish(self.throttle_home)
			self.actuator_pub.publish(self.actuator_home)

			self.start_path_following(path_array)



	def rover_position_callback(self, msg):
		"""
		Position from GPS converted to UTM.

		It also checks the rover's distance between its current
		position and the goal position in the course. Moves to
		next goal in course if rover is within look ahead distance.

		NOTE: Does the target index need to increment here and/or turn loop?

		"""
		# print("Rover position callback: {}".format(msg))

		# if not self.current_goal:
		# 	return  # wait until a goal is set

		
		_lat, _lon = msg.latitude, msg.longitude
		curr_pose_utm = utm.from_latlon(_lat, _lon)
		self.current_pos = [curr_pose_utm[0], curr_pose_utm[1]]



	def rover_imu_callback(self, msg):
		"""
		Angle from IMU in radians.
		"""
		self.current_angle = self.quat_to_angle(msg.orientation)



	def quat_to_angle(self, quat):
		"""
		Converts quaternion to angle.
		"""
		rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
		return rot.GetRPY()[2]



	def start_path_following(self, path_array):

		if not isinstance(path_array, list):
			self.shutdown()
			raise Exception("Path must be a list of [easting, northing] pairs..")

		if len(path_array) < 1:
			self.shutdown()
			raise Exception("Path must be at least one point..")


		while not self.current_pos:
			rospy.sleep(1)
			print("Waiting for GPS data from /fix topic..")


		self.np_course = np.array(path_array)  # sets numpy array of course


		rospy.sleep(2)  # give messages time to publish


		_curr_utm = self.current_pos
		self.target_index = self.calc_target_index(_curr_utm, 0, self.np_course[:,0], self.np_course[:,1])
		self.current_goal = self.path_array[self.target_index]  # sets current goal


		print("Initial target index: {}".format(self.target_index))
		print("Total length of path array: {}".format(len(self.path_array)))


		_curr_utm = self.current_pos
		# self.target_index = self.calc_target_index(_curr_utm, 0, self.np_course[:,0], self.np_course[:,1])
		# self.current_goal = self.path_array[self.target_index]  # sets current goal

		print("Initial target goal index: {}".format(self.target_index))
		print("Initial goal: {}".format(self.current_goal))


		# Sleep routine for testing:
		print("Pausing 10 seconds before initiating driving (to have time to run out there)...")
		rospy.sleep(10)
		print("Starting driving routine.")



		print(">>> Starting drive actuator to drive foward!")
		self.throttle_pub.publish(self.throttle_drive_slow)  # sets to 100

		rospy.sleep(2)

		self.actuator_pub.publish(self.actuator_drive_slow)  # sets to 20




		# return




		###################################################################
		# This loop calculates a turn angle a look-ahead distance away,
		# then begins to execute the turn.
		###################################################################
		while not rospy.is_shutdown() and not self.at_flag:

			if self.at_flag:
				print("At a flag in the course! Stopping the rover to take a sample.")
				rospy.sleep(0.1)
				self.articulator_pub.publish(self.actuator_stop)
				break


			# rospy.sleep(1/self.rate)  # sleep for 100ms
			rospy.sleep(0.2)


			_curr_utm = self.current_pos  # gets current utm

			print("Current UTM in nav loop: {}".format(_curr_utm))
			
			# using goal a look ahead away for calculating turn angle

			print("Calculating new target index..")

			# _target_index = self.calc_target_index(_curr_utm, self.target_index, self.np_course)
			# self.target_index = self.calc_target_index(_curr_utm, self.target_index, self.np_course[:,0], self.np_course[:,1])

			self.target_index = self.calc_target_index(_curr_utm, self.target_index, self.np_course[:,0], self.np_course[:,1])


			# print("target index: {}".format(self.target_index))
			print("target index: {}".format(self.target_index))


			if self.target_index == None:
				print("Assuming end of course is reached! Stopping rover.")
				self.shutdown()

			self.current_goal = self.path_array[self.target_index]
			_curr_angle = self.current_angle  # gets current angle in radians

			A = (_curr_utm[0], _curr_utm[1], _curr_angle)
			B = (self.current_goal[0], self.current_goal[1], 0)  # note: B angle not used..

			# note: flipped sign of turn from imu
			turn_angle = -1.0*orientation_transforms.initiate_angle_transform(A, B)

			# print("Turn angle: {}".format(turn_angle))

			if abs(turn_angle) > abs(self.angle_tolerance):

				if turn_angle < -self.angle_trim:
					turn_angle = -self.angle_trim

				elif turn_angle > self.angle_trim:
					turn_angle = self.angle_trim

				print("Telling Rover to turn {} degreess..".format(turn_angle))
				# self.translate_angle_with_imu(turn_angle)  # note: in degrees, converted to radians in nav_controller
				self.translate_angle_with_imu(turn_angle)
				print("Finished turn.")


		if self.at_flag:
			# broke out of drive loop because it's at the flag.
			# make sure actuator is set to stop.
			print("Making sure rover is stopped, then making request to take a sample..")
			rospy.sleep(0.1)
			self.actuator_pub.publish(self.actuator_stop)

			# call sample collector service here..
			print("Pausing 10s to simulate a sample collection routine..")
			rospy.sleep(10)

			_curr_utm = self.current_pos
			self.target_index = self.calc_target_index(_curr_utm, self.target_index, self.np_course[:,0], self.np_course[:,1])

			print("New target index: {}".format(self.target_index))

			rospy.sleep(1)

			updated_path = self.path_array[self.target_index:]  # set remaining path to follow

			rospy.sleep(1)

			self.at_flag = False  # set at_flag to False after sample is collected..

			# start following path again:
			print("Starting path following again..")
			rospy.sleep(1)
			self.start_path_following(updated_path)





		print("Finished driving course..")
		print("Shutting down Jackal..")
		self.shutdown()
		


	def calc_target_index(self, current_position, current_goal_index, cx, cy):
		"""
		From red_rover_model pure_puruit module. Loops through course
		points (x and y) and builds a list of the diff b/w robot's position and
		each x and y in t{he course. Finally, 
				"""
		# print("Inside calc target index function.")
		# print("Current poisition: {}".format(current_position))
		# print("Current goal index: {}".format(current_goal_index))

		# note: numpy seems to return blank array if out of index, so
		# it should return None at end of course.

		dx = [current_position[0] - icx for icx in cx[current_goal_index:]]  # diff b/w robot's position and all x values in course (starting at current goal, onward)
		dy = [current_position[1] - icy for icy in cy[current_goal_index:]]  # diff b/w robot's position and all y values in course (starting at current goal, onward)

		d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]  # scalar diff b/w robot and course values

		print("Determining goal point based on look-ahead of {}".format(self.look_ahead))

		# print("distances: {}".format(d))

		ind = 0
		for pos_diff in d:
			if pos_diff > self.look_ahead:

				# below gets the index in this segment of course, but the
				# index relative to the whole course needs to be returned..
				# return d.index(pos_diff)  # return index of goal to go to

				_index = d.index(pos_diff)
				goal_index = current_goal_index + _index  # return index of goal in the whole course

				return goal_index



			ind += 1

		return None



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
		# last_angle = self.get_jackal_rot().jackal_rot  # get angle from IMU (in radians)
		last_angle = self.current_angle

		# while abs(turn_angle) < abs(goal_angle) and not self.at_flag and not rospy.is_shutdown():
		while abs(turn_angle) < abs(radians(goal_angle)) and not self.at_flag and not rospy.is_shutdown():

			# print("Current angle: {}, Current pivot: {}".format(self.last_angle, self.current_pivot))

			self.articulator_pub.publish(_turn_val)

			rospy.sleep(1.0/self.rate)

			# curr_angle = self.get_jackal_rot().jackal_rot
			curr_angle = self.current_angle
			delta_angle = self.normalize_angle(curr_angle - last_angle)
			turn_angle += delta_angle
			last_angle = curr_angle

			if delta_angle == 0.0:
				break

		self.articulator_pub.publish(self.no_turn_val)  # stop turning once goal angle is reached.

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
		course_filename = sys.argv[1]
	except IndexError:
		raise IndexError("Course not specified. Add course filename as arg when running basic_drive_5.py")

	coursefile = open(course_filename, 'r')
	course = json.loads(coursefile.read())

	try:
		SingleGoalNav(course)
	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation terminated.")
		rospy.loginfo("Shutting down drive node!")
		raise Exception("basic drive ROS node exception")

	rospy.spin()