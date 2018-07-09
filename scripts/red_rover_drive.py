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
		# rospy.Subscriber("/fix", NavSatFix, self.rover_position_callback, queue_size=1)
		# rospy.Subscriber('/phidget/imu/data', Imu, rot_callback_imu, queue_size=1)
		
		# Set rospy to exectute a shutdown function when terminating the script
		rospy.on_shutdown(self.shutdown)

		# How fast will we check the odometry values?
		self.rate = 10
		
		# Set the equivalent ROS rate variable
		self.r = rospy.Rate(self.rate)

		# self.nav_controller = NavController()  # module that handles driving and turning routines

		self.angle_tolerance = 1.0  # angle tolerance in degrees

		self.path_json = path_json  # The path/course the red rover will follow!

		self.look_ahead = 5.0
		self.min_position_tolerance = 0.2  # min distance from goal to move on to next one
		self.distance_from_goal = 0.0

		self.current_goal = [0,0]  # [easting, northing] array
		self.current_pos = [0,0]  # [easting, northing] array
		self.current_angle = 0.0



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


			print("Setting throttle and drive actuator to home states..")
			nc.throttle_pub.publish(nc.throttle_home)
			nc.actuator_pub.publish(nc.actuator_home)


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



		print(">>> Starting drive actuator to drive foward!")
		nc.throttle_pub.publish(nc.throttle_drive_slow)  # sets to 100
		nc.actuator_pub.publish(nc.actuator_drive_slow)  # sets to 20



		# Loop track goals here for each A->B in the course:
		for i in range(target_index, len(path_array) - 1):
		
			target_index = i

			print ("i: {}".format(i))
			self.current_goal = path_array[i]

			curr_pose_utm = nc.get_current_position()  # returns NavSatFix type of position
			curr_angle = nc.get_jackal_rot().jackal_rot  # returns float64 of angle in radians, i think

			print("Angle from jackal rotation service (i.e., not odom rotation value): {}".format(curr_angle))
			print("Same angle, but in degrees: {}".format(degrees(curr_angle)))

			transformed_angle = orientation_transforms.transform_imu_frame(degrees(curr_angle))

			A = (curr_pose_utm[0], curr_pose_utm[1], radians(transformed_angle))  # initial position of jackal
			B = (self.current_goal[0], self.current_goal[1], 0)  # note: B orientation is irrelevant

			drive_distance = self.determine_drive_distance(A, B)  # get drive distance from current position to goal

			# Skips to next goal/course point if said goal is less than look-ahead:
			if drive_distance < self.look_ahead:
				print("Within look-ahead of goal, so skipping to next goal")
				continue  # skip to next iteration in track for loop

			self.current_goal = B  # TODO: organize this..
			self.p2p_drive_routine(self.current_goal, target_index, _np_track)  # main drive routine0


		print("Finished driving course..")
		print("Shutting down Jackal..")
		self.shutdown()



	def p2p_drive_routine(self, goal_pos, target_index, np_course):
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

		turn_angle = orientation_transforms.initiate_angle_transform(A, B)
		print("Turn angle, pre-tolerance filter: {}".format(turn_angle))
		turn_angle = -1.0 * turn_angle

		print("Flipping direction around as it's the inverse of what is expected!!!")
		print("New angle: {}".format(turn_angle))

		if abs(turn_angle) > abs(self.angle_tolerance):

			if turn_angle < -10:
				turn_angle = -10
			elif turn_angle > 10:
				turn_angle = 10

			# Determine angle to turn based on IMU..
			print("Telling Jackal to turn {} degreess..".format(turn_angle))
			nc.translate_angle_with_imu(turn_angle)  # note: in degrees, converted to radians in nav_controller
			print("Finished turn.")


		curr_pose_utm = nc.get_current_position()
		A = (curr_pose_utm[0], curr_pose_utm[1], curr_angle)
		drive_distance = self.determine_drive_distance(A, B)
		print("Initiating drive loop.. Drive distance: {}".format(drive_distance))

		original_turn_angle = turn_angle
		original_drive_distance = drive_distance


		loop_period = 1.0 / self.rate
		turn_check_counter = 0

		while drive_distance > self.min_position_tolerance:

			print("Drive distance to goal: {}".format(drive_distance))

			rospy.sleep(loop_period)

			curr_pose_utm = nc.get_current_position()
			A = (curr_pose_utm[0], curr_pose_utm[1], curr_angle)
			drive_distance = self.determine_drive_distance(A, B)

			if turn_check_counter % self.rate:
				# hits this every conditional every 1 second
				print("Checking turn angle relative to look-ahead..")
				# calculates index to a new look ahead to calculate turn angle:
				target_index = self.calc_target_index(A, target_index, np_course[:,0], np_course[:,1])

				A = (curr_pose_utm[0], curr_pose_utm[1], curr_angle)
				B = (goal_pos[0], goal_pos[1], goal_pos[2])  # NOTE: B's orientation currently hardcoded for testing..

				future_goal = self.path_array[target_index]  # get goal look-ahead away to determine turn

				turn_angle = orientation_transforms.initiate_angle_transform(A, self.path_array[target_index])
				turn_angle = -1.0 * turn_angle

				if abs(turn_angle) > abs(self.angle_tolerance):
					
					if turn_angle < -22:
						turn_angle = -10
					elif turn_angle > 22:
						turn_angle = 10

					# Determine angle to turn based on IMU..
					print("Telling Jackal to turn {} degreess..".format(turn_angle))
					nc.translate_angle_with_imu(turn_angle)  # note: in degrees, converted to radians in nav_controller
					print("Finished turn.")




			if drive_distance > original_drive_distance + 0.1:
				# if the drive distance starts to grow instead of shrink, move to next goal point
				print("Moving away from goal point.. breaking out of drive loop and looking at next goal..")
				break

			turn_check_counter += 1


		print("Arrived at course goal position..")
		print("Moving on to next goal position..")
		


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