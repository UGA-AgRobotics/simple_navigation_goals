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
from simple_navigation_goals.srv import *
import math
import json
from math import radians, copysign, sqrt, pow, pi, degrees
import PyKDL
import numpy as np

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
		
		# Set rospy to exectute a shutdown function when terminating the script
		rospy.on_shutdown(self.shutdown)

		# How fast will we check the odometry values?
		rate = 20
		
		# Set the equivalent ROS rate variable
		self.r = rospy.Rate(rate)

		# self.nav_controller = NavController()  # module that handles driving and turning routines

		self.angle_tolerance = 2.0  # angle tolerance in degrees

		self.path_json = path_json  # The path/course the red rover will follow!

		self.look_ahead = 1.0



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
			
			print("The Course: {}".format(path_array))
			print("Starting path following routine..")

			self.start_path_following(path_array)



	def start_path_following(self, path_array):


		_np_track = np.array(path_array)  # get np array of path for easy manipulations


		# Sleep routine for testing:
		print("Pausing 10 seconds before initiating driving (to have time to run out there)...")
		rospy.sleep(10)
		print("Starting driving routine.")


		# Initial GPS position:
		curr_pose_utm = nc.get_current_position()

		# Accounting for look-ahead distance:
		target_index = self.calc_target_index(curr_pose_utm, 0, _np_track[:,0], _np_track[:,1])
		print("Initial target goal index: {}".format(target_index))



		# Loop track goals here for each A->B in the course:
		# for i in range(target_index, len(path_array) - 1):
		for i in range(0,1):

			print ("i: {}".format(i))
			current_goal = path_array[i]

			future_goal = None
			try:
				future_goal = path_array[i + 1]
			except IndexError as e:
				print("Current goal is the last one in the course!")

			goal_orientation = orientation_transforms.determine_angle_at_goal(current_goal, future_goal)



			curr_pose_utm = nc.get_current_position()  # returns NavSatFix type of position
			curr_angle = nc.get_jackal_rot().jackal_rot  # returns float64 of angle in radians, i think

			print("Angle from jackal rotation service (i.e., not odom rotation value): {}".format(curr_angle))
			print("Same angle, but in degrees: {}".format(degrees(curr_angle)))


			transformed_angle = orientation_transforms.transform_imu_frame(degrees(curr_angle))

			A = (curr_pose_utm[0], curr_pose_utm[1], radians(transformed_angle))  # initial position of jackal
			B = (current_goal[0], current_goal[1], goal_orientation)


			drive_distance = self.determine_drive_distance(A, B)  # get drive distance from current position to goal

			# Skips to next goal/course point if said goal is less than look-ahead:
			if drive_distance < self.look_ahead:
				print("Within look-ahead of goal, so skipping to next goal")
				continue  # skip to next iteration in track for loop

			current_goal = B  # TODO: organize this..
			self.p2p_drive_routine(current_goal)  # main drive routine


			# NOTE: IS THIS SECTION ACTUALLY NEEDED/USEFUL??
			#####################################################################################################
			curr_pose_utm = nc.get_current_position()
			new_target_index = self.calc_target_index(curr_pose_utm, target_index, _np_track[:,0], _np_track[:,1])
			print("Target index: {}".format(new_target_index))
			if new_target_index > target_index:
				print(">>> Jackal is within look-ahead distance of goal, starting to drive toward next goal now..")
				target_index = new_target_index
				i = target_index
				print ("@@@ new i: {} @@@".format(i))
			#####################################################################################################

			print("STOPPING SINGLE POINT TEST!")
			return  # REMOVE THIS AFTER SINGLE GOAL TESTING !!!!!!!!!!!!!


		print("Shutting down Jackal..")
		self.shutdown()



	def p2p_drive_routine(self, goal_pos):
		"""
		The drive routine from point-to-point, whether that's b/w
		two GPS points on the course, or a step size incrementing a drive
		between two GPS points.
		"""
		curr_pose_utm = nc.get_current_position()
		curr_angle = nc.get_jackal_rot().jackal_rot

		print("Jackal's position in UTM: {}, Jackal's angle: rad-{}, deg-{}".format(curr_pose_utm, curr_angle, degrees(curr_angle)))
		print("GOAL POSITION: {}".format(goal_pos))

		A = (curr_pose_utm[0], curr_pose_utm[1], curr_angle)
		B = (goal_pos[0], goal_pos[1], goal_pos[2])  # NOTE: B's orientation currently hardcoded for testing..

		# NOTE: perhaps add angle tolerance here, e.g., if turn_angle is
		# 0 +/- angle_tolerance, then don't turn the rover!
		##########################################################################
		turn_angle = orientation_transforms.initiate_angle_transform(A, B)

		print("Turn angle, pre-tolerance filter: {}".format(turn_angle))

		if abs(turn_angle) > abs(self.angle_tolerance):
		# if turn_angle != 0:
			# Determine angle to turn based on IMU..
			print("Telling Jackal to turn {} degreess..".format(turn_angle))
			# nc.execute_turn(radians(turn_angle))
			nc.translate_angle_with_imu(turn_angle)  # note: in degrees, converted to radians in nav_controller
			print("Finished turn.")
		##########################################################################

		# drive_distance = self.determine_drive_distance(A, B)

		# if drive_distance > 0:
		# 	nc.drive_forward(drive_distance, self.look_ahead)



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
		rospy.loginfo(">>>>> Stopping the robot by publishing blank Twist to jackal_nav_controller..")
		nc.shutdown_all()
		# nt.shutdown_all()
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