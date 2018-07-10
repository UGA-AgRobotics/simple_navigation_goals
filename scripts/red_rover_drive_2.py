#!/usr/bin/env python

"""
This drive routine is the same as basic_drive_5.py, but taking out the odom and base frames
that I believe are related to the turtlebot. The course will be tested again (course 9, etc.)
using just the IMU and GPS for navigation.
"""



import roslib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
from simple_navigation_goals.srv import *
import math
import json
from math import radians, copysign, sqrt, pow, pi, degrees
import PyKDL
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

# Local package requirements:
from nav_tracks import NavTracks
from red_rover_nav_controller import NavController
import orientation_transforms



nc = NavController()



class SingleGoalNav(object):
	"""
	Testing Jackal navigation to a single goal/flag. Determines
	X and Y distance to travel using its GPS location and flag's 
	location, both converted to UTM.
	"""

	# nc = NavController()


	def __init__(self, path_json=None):
		
		# Give the node a name
		rospy.init_node('single_goal_nav')

		# Subscribers:
		rospy.Subscriber("/start_driving", Bool, self.start_driving_callback, queue_size=1)
		rospy.Subscriber("/fix", NavSatFix, self.rover_position_callback, queue_size=1)
		rospy.Subscriber('/phidget/imu/data', Imu, self.rover_imu_callback, queue_size=1)
		
		# Set rospy to exectute a shutdown function when terminating the script
		rospy.on_shutdown(self.shutdown)

		# How fast will we check the odometry values?
		self.rate = 10
		
		# Set the equivalent ROS rate variable
		self.r = rospy.Rate(self.rate)

		# self.nav_controller = NavController()  # module that handles driving and turning routines

		self.angle_tolerance = 0.5  # angle tolerance in degrees

		self.path_json = path_json  # The path/course the red rover will follow!
		self.path_array = None  # path converted to list of [easting, northing]

		self.look_ahead = 3.0

		self.min_position_tolerance = 0.2  # min distance from goal to move on to next one
		self.distance_from_goal = 0.0

		self.angle_trim = 5.0  # (in degrees)

		self.target_index = None  # index in course that's the goal position
		self.current_goal = None  # [easting, northing] array
		self.current_pos = None  # [easting, northing] array
		self.current_angle = None  # angle from imu in radians

		self.np_course = None  # lazy np array version of course for certain manipulations



	def start_driving_callback(self, msg):
		"""
		Initiates driving routine.
		The course file that was referenced when initiating the RedRoverDrive class
		is converted to a list of [easting, northing] pairs, then initiate the rover
		to drive and follow the course.
		"""
		if msg.data == True:

			# Gets track to follow:
			nt = NavTracks()
			path_array = nt.get_track_from_course(self.path_json)  # builds list of [easting, northing] pairs from course file
			
			self.path_array = path_array

			print("The Course: {}".format(path_array))
			print("Starting path following routine..")


			print("Setting throttle and drive actuator to home states..")
			nc.throttle_pub.publish(nc.throttle_home)
			nc.actuator_pub.publish(nc.actuator_home)


			self.start_path_following(path_array)



	def rover_position_callback(self, msg):
		"""
		Position from GPS converted to UTM.

		It also checks the rover's distance between its current
		position and the goal position in the course. Moves to
		next goal in course if rover is within look ahead distance.
		"""

		if not self.current_goal:
			return  # wait until a goal is set

		
		_lat, _lon = msg.latitude, msg.longitude
		curr_pose_utm = utm.from_latlon(_lat, _lon)
		self.current_pos = [curr_pose_utm[0], curr_pose_utm[1]]


		# compare current pos with current goal:

		_curr_utm = self.current_pos  # gets current position from GPS
		_curr_goal = self.current_goal  # gets current goal in course
		drive_distance = self.determine_drive_distance(_curr_utm, _curr_goal)  # gets distance b/w

		if drive_distance < self.look_ahead:

			print("target index, path array length: {}, {}".format(self.target_index, self.path_array))

			# check to see if at end of course:
			if self.target_index >= len(self.path_array) - 1:
				# need the -1?
				print("At end of course! Stopping the rover.")
				self.shutdown()
				return


			print("Drive distance is less than look ahead, setting new goal in course.")
			print("GPS callback incrementing target index.")
			self.target_index += 1  # increments target index
			self.current_goal = self.path_array[self.target_index]  # sets new goal
			print("New goal: {}".format(self.current_goal))



		# if within look-ahead, what do next?


		# do flag stuff here? or keep flag node?




	def rover_imu_callback(self, msg):
		"""
		Angle from IMU in radians.
		"""
		self.current_angle = quat_to_angle(msg.orientation)




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



		_curr_utm = self.current_pos
		self.target_index = self.calc_target_index(_curr_utm, 0, self.np_course[:,0], self.np_course[:,1])
		self.current_goal = self.path_array[self.target_index]  # sets current goal

		print("Initial target goal index: {}".format(self.target_index))
		print("Initial goal: {}".format(self.current_goal))


		# Sleep routine for testing:
		print("Pausing 10 seconds before initiating driving (to have time to run out there)...")
		rospy.sleep(10)
		print("Starting driving routine.")



		print(">>> Starting drive actuator to drive foward!")
		nc.throttle_pub.publish(nc.throttle_drive_slow)  # sets to 100

		rospy.sleep(2)

		nc.actuator_pub.publish(nc.actuator_drive_slow)  # sets to 20


		###################################################################
		# This loop calculates a turn angle a look-ahead distance away,
		# then begins to execute the turn.
		###################################################################
		while not rospy.is_shutdown():

			# rospy.sleep(1/self.rate)  # sleep for 100ms
			rospy.sleep(0.5)


			_curr_utm = self.current_pos  # gets current utm
			
			# using goal a look ahead away for calculating turn angle

			_target_index = self.calc_target_index(_curr_utm, self.target_index, self.np_course)
			_turn_goal = self.path_array[_target_index]

			_curr_angle = self.current_angle  # gets current angle in radians

			A = (_curr_utm[0], _curr_utm[1], _curr_angle)
			B = (_turn_goal[0], _turn_goal[1], 0)  # note: B angle not used..

			# note: flipped sign of turn from imu
			turn_angle = -1.0*orientation_transforms.initiate_angle_transform(A, B)

			print("Turn angle: {}".format(turn_angle))

			if abs(turn_angle) > abs(self.angle_tolerance):

				if turn_angle < -self.angle_trim:
					turn_angle = -self.angle_trim

				elif turn_angle > self.angle_trim:
					turn_angle = self.angle_trim

				print("Telling Jackal to turn {} degreess..".format(turn_angle))
				nc.translate_angle_with_imu(turn_angle)  # note: in degrees, converted to radians in nav_controller
				print("Finished turn.")


		print("Finished driving course..")
		print("Shutting down Jackal..")
		self.shutdown()
		


	def calc_target_index(self, current_position, current_goal_index, cx, cy):
		"""
		From red_rover_model pure_puruit module. Loops through course
		points (x and y) and builds a list of the diff b/w robot's position and
		each x and y in the course. Finally, 
		"""
		dx = [current_position[0] - icx for icx in cx[current_goal_index:]]  # diff b/w robot's position and all x values in course (starting at current goal, onward)
		dy = [current_position[1] - icy for icy in cy[current_goal_index:]]  # diff b/w robot's position and all y values in course (starting at current goal, onward)

		d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]  # scalar diff b/w robot and course values

		print("Determining goal point based on look-ahead of {}".format(self.look_ahead))

		ind = 0
		for pos_diff in d:
			# print("Distance between Jackal and goal: index={}, value=({}, {}), distance={} meters".format(ind, cx[ind], cy[ind], d[ind]))
			print("Distance between Jackal and goal: index={}, value=({}, {}), distance={} meters".format(ind, cx[ind], cy[ind], pos_diff))
			if pos_diff > self.look_ahead:
				return d.index(pos_diff)  # return index of goal to go to
			ind += 1



	def determine_drive_distance(self, A, B):
		return math.sqrt((B[1] - A[1])**2 + (B[0] - A[0])**2)




	def shutdown(self):
		"""
		Always stop the robot when shutting down the node
		"""
		nc.shutdown_all()
		rospy.sleep(1)







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