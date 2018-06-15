#!/usr/bin/env python

"""
This is the same as basic_drive_4.py, except it's going to not use the dubins model
between course points (tightly-spaced gps points from emlid reach unit). A look-ahead
will still be used so the Jackal isn't trying to hit every single position as it follows
the path.

TODO: Still need to move actually turning and driving execution to this file instead of
in the service modules (jackal_pos_server and jackal_rot_server)
"""



import roslib
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from std_msgs.msg import Bool
from simple_navigation_goals.srv import *
import tf
import math
import json
from math import radians, copysign, sqrt, pow, pi, degrees
import utm
import PyKDL
import numpy as np
import matplotlib.pyplot as plt
import dubins

# Local package requirements:
from nav_tracks import NavTracks
# import jackal_nav_controller  # handles drive and turn routines
from jackal_nav_controller import NavController



class SingleGoalNav():
	"""
	Testing Jackal navigation to a single goal/flag. Determines
	X and Y distance to travel using its GPS location and flag's 
	location, both converted to UTM.
	"""

	def __init__(self, course=None):
		
		# Give the node a name
		rospy.init_node('single_goal_nav', anonymous=False)
		
		# Set rospy to exectute a shutdown function when terminating the script
		rospy.on_shutdown(self.shutdown)

		# How fast will we check the odometry values?
		rate = 20
		
		# Set the equivalent ROS rate variable
		self.r = rospy.Rate(rate)
		

		self.at_flag = False  # keeps track of if robot is at a flag or not




		self.nav_controller = NavController()



		# rospy.Subscriber("at_flag", Bool, self.flag_callback)  # sub to /at_flag topic







		
		# Publisher to control the robot's speed
		self.cmd_vel = rospy.Publisher('/cmd_vel', Twist)
		 
		# The base frame is base_footprint for the TurtleBot but base_link for Pi Robot
		self.base_frame = rospy.get_param('~base_frame', '/base_link')

		# The odom frame is usually just /odom
		self.odom_frame = rospy.get_param('~odom_frame', '/odom')

		# Initialize the tf listener
		self.tf_listener = tf.TransformListener()
		
		# Find out if the robot uses /base_link or /base_footprint
		try:
			print("Waiting for TF between /odom and /base_footprint..")
			self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
			self.base_frame = '/base_footprint'
		except (tf.Exception, tf.ConnectivityException, tf.LookupException):
			try:
				print("Trying TF between /odom and /base_link..")
				self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
				self.base_frame = '/base_link'
			except (tf.Exception, tf.ConnectivityException, tf.LookupException):
				rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
				rospy.signal_shutdown("tf Exception")





		# ALL THIS IS TEMPORARILY COMMENTED OUT FOR TESTING THE FLAG NODE!!!!
		# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


		position = Point()  # initialize the position variable as a Point type
		move_cmd = Twist()  # initialize movement comment

		# move_cmd.linear.x = linear_speed  # set movement command to forward motion

		(position, rotation) = self.get_odom()  # get starting position values



		###################################################################
		# Gets track to follow:											  #
		###################################################################
		nt = NavTracks()

		if not course:
			raise Exception("Must specify course")

		# _track = nt.get_track('track4')  # just one initial goal point for initial dubins testing..
		_track = nt.get_track_from_course(course)  # builds track from a GPS course/path
		_np_track = np.array(_track)

		print("The Course: {}".format(_track))
		###################################################################



		# Main model constants (todo: move main model constants to top):
		######################################################################
		# qs_array = []  # collection of points for dubins paths
		# turning_radius = 2.5  # min turning radius for robot in meters
		# step_size = 0.5  # dubins model step size in meters
		self.look_ahead = 1.0  # look-ahead distance in meters
		######################################################################



		# # Sleep routine for testing:
		# print("Pausing 10 seconds before initiating driving (to have time to run out there)...")
		# rospy.sleep(10)
		# print("Initiating driving to point B..")



		# Initial GPS position:
		curr_pose_utm = self.get_current_position()

		# Accounting for look-ahead distance:
		target_index = self.calc_target_index(curr_pose_utm, 0, _np_track[:,0], _np_track[:,1])
		print("Initial target goal index: {}".format(target_index))


		# Loop track goals here for each A->B in the course:
		for i in range(target_index, len(_track) - 1):

			print ("i: {}".format(i))
			current_goal = _track[i]

			future_goal = None
			try:
				future_goal = _track[i + 1]
			except IndexError as e:
				print("Current goal is the last one in the course!")
				print("End at the same orientation as the last goal..")
				pass  # continue on..
			goal_orientation = self.determine_angle_at_goal(current_goal, future_goal)

			curr_pose_utm = self.get_current_position()

			transformed_angle = self.transform_imu_frame(degrees(rotation))

			A = (curr_pose_utm[0], curr_pose_utm[1], math.radians(transformed_angle))  # initial position of jackal
			B = (current_goal[0], current_goal[1], goal_orientation)

			x_diff = B[0] - A[0]
			y_diff = B[1] - A[1]

			AB_theta0 = math.atan2(abs(y_diff), abs(x_diff))  # get intitial angle, pre transform
			AB_angle = self.transform_angle_by_quadrant(AB_theta0, x_diff, y_diff)  # determine angle between vector A and B


			drive_distance = self.determine_drive_distance(A, B)  # get drive distance from current position to goal

			# Skips to next goal/course point if said goal is less than look-ahead:
			if drive_distance < self.look_ahead:
				print("Within look-ahead of goal, so skipping to next goal")
				continue  # skip to next iteration in track for loop



			print("Executing drive routine..")
			print("Robot's at_flag val: {}".format(self.at_flag))
			# while not self.at_flag:
			# 	self.p2p_drive_routine(current_goal)

			self.p2p_drive_routine(current_goal)

			curr_pose_utm = self.get_current_position()

			new_target_index = self.calc_target_index(curr_pose_utm, target_index, _np_track[:,0], _np_track[:,1])

			print("Target index: {}".format(new_target_index))

			if new_target_index > target_index:
				print(">>> Jackal is within look-ahead distance of goal, starting to drive toward next goal now..")
				target_index = new_target_index
				i = target_index
				print ("new i: {}".format(i))
				continue



		print("Shutting down Jackal..")
		self.shutdown()

		# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



	def p2p_drive_routine(self, goal_pos):
		"""
		The drive routine from point-to-point, whether that's b/w
		two GPS points on the course, or a step size incrementing a drive
		between two GPS points.
		"""
		(position, rotation) = self.get_odom()  # get starting position values
		curr_pose_utm = self.get_current_position()

		print("Jackal's position in UTM: {}".format(curr_pose_utm))

		A = (curr_pose_utm[0], curr_pose_utm[1], rotation)
		B = (goal_pos[0], goal_pos[1], rotation)  # NOTE: B's orientation currently hardcoded for testing..
		
		turn_angle = self.initiate_angle_transform(A, B)

		if turn_angle != 0:
			# Determine angle to turn based on IMU..
			print("Telling Jackal to turn {} degreess..".format(turn_angle))
			# self.call_jackal_rot_service(turn_angle)
			# jackal_nav_controller.execute_turn(radians(turn_angle))
			self.nav_controller.execute_turn(radians(turn_angle))
			print("Finished turning..")

		drive_distance = self.determine_drive_distance(A, B)

		if drive_distance > 0:
			print("Driving Jackal {} meters..".format(drive_distance))
			# self.get_jackal_pos_from_service(drive_distance)
			# jackal_nav_controller.drive_forward(drive_distance, self.look_ahead)
			self.nav_controller.drive_forward(drive_distance, self.look_ahead)
			print("Finished driving..")



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



	def get_current_position(self):
		curr_pose = self.get_jackal_pos_from_service()  # don't drive, just get current lat/lon
		curr_pose_utm = utm.from_latlon(curr_pose.jackal_fix.latitude, curr_pose.jackal_fix.longitude)
		return curr_pose_utm



	def determine_angle_at_goal(self, goal, future_goal):
		"""
		Calculates the angle of orientation for the robot to be in when
		it gets to its goal. Creates a vector that points to the future goal.
		If there is no future goal, end in the same orientation, just end at 
		the goal in the orientation it arrives in.
		"""
		if not future_goal:
			return None

		x_diff = future_goal[0] - goal[0]
		y_diff = future_goal[1] - goal[1]

		return math.atan2(y_diff, x_diff)  # returns full angle relative to easting,north -> x,y axis



	def build_goals_from_dubins(self, dubins_path):
		"""
		Takes a dubins array (a numpy array), and builds a list
		of list, where the latter items are easting, northing values (like nav_tracks.py)
		"""
		x_array = dubins_path[:,0]
		y_array = dubins_path[:,1]
		goals_array = []
		for i in range(0, len(dubins_path)):
			xy_pair = [x_array[i], y_array[i]]
			goals_array.append(xy_pair)
		return goals_array



	def plot_full_dubins_path(self, qs_array, x_path=None, y_path=None, show=True, filename=None):
		"""
		Like plot_dubins_path() function, but plots a full set of points
		instead a single A -> B two point dataset.
		"""
		# Initial setup: No directional plotting, just dots and path at the moment..

		for qs in qs_array:
			self.plot_dubins_path(qs['qs'], qs['q0'], qs['q1'], show=False)

		plt.plot(x_path, y_path, 'bo')  # overlay path points onto plot

		if not show and filename:
			print("Saving plot named {}".format(filename))
			plt.savefig('{}.png'.format(filename))  # for plots across look-aheads
			print("Plot saved.")
		else:
			plt.show()  # display plot



	def plot_dubins_path(self, qs, q0, q1, show=True):
		"""
		Plots the dubins path between a starting and ending point.
		Inputs:
			qs - dubins path data [[x,y,angle], ..]
			q0 - initial position [x,y,angle]
			q1 - target position [x,y,angle]
		Returns: None
		"""
		xs = qs[:,0]
		ys = qs[:,1]
		us = xs + np.cos(qs[:, 2])
		vs = ys + np.sin(qs[:, 2])
		plt.plot(q0[0], q0[1], 'gx', markeredgewidth=4, markersize=10)  # actual start point
		plt.plot(q1[0], q1[1], 'rx', markeredgewidth=4, markersize=10)  # actual end point
		plt.plot(xs, ys, 'b-')
		plt.plot(xs, ys, 'r.')
		plt.plot(qs[0][0], qs[0][1], 'go', markersize=5)  # dubins start point
		plt.plot(qs[-1][0], qs[-1][1], 'ro', markersize=5)  # dubins end point

		for i in xrange(qs.shape[0]):
			plt.plot([xs[i], us[i]], [ys[i], vs[i]],'r-')

		if show:
			plt.show()



	def initiate_angle_transform(self, A, B):
		"""
		Transforms angle between the IMU frame (magnetic North) and
		the Jackal's frame. Takes in angle from IMU, then determines
		an angle and turn direction for the Jackal to execute.
		"""
		x_diff = B[0] - A[0]
		y_diff = B[1] - A[1]

		_trans_angle = self.transform_imu_frame(degrees(A[2]))
		AB_theta0 = math.atan2(abs(y_diff), abs(x_diff))  # get intitial angle, pre transform
		AB_angle = self.transform_angle_by_quadrant(AB_theta0, x_diff, y_diff)  # determine angle between vector A and B

		turn_angle = None
		if AB_angle == 0:
			turn_angle = 0
		else:
			turn_angle = AB_angle - _trans_angle  # angle to turn (signage should denote direction to turn)

		return turn_angle  # return angle to turn relative to jackal's current orientation



	def transform_angle_by_quadrant(self, initial_angle, x_diff, y_diff):
		"""
		Takes the change in X and Y to determine how the Jackal
		should turn.
		"""
		if x_diff > 0 and y_diff > 0:
			print("p1 in quadrant: {}".format(1))
			# Point B in quadrant 1..
			return degrees(initial_angle)
		elif x_diff < 0 and y_diff > 0:
			print("p1 in quadrant: {}".format(2))
			# Point B in quadrant 2..
			return 180 - degrees(initial_angle)
		elif x_diff < 0 and y_diff < 0:
			print("p1 in quadrant: {}".format(3))
			# Point B in quadrant 3..
			return 180 + degrees(initial_angle)
		elif x_diff > 0 and y_diff < 0:
			print("p1 in quadrant: {}".format(4))
			# Point B in quadrant 4..
			return 360 - degrees(initial_angle)
		elif x_diff == 0 and y_diff == 0:
			# No change in angle..
			return 0.0
		else:
			raise Exception("!Error occurred in basic_drive_3/transform_angle_by_quadrant func..")



	def transform_imu_frame(self, theta0):
		"""
		Transform initial robot angle in IMU frame to have 0 degrees
		in East direction, going 0->360 CCW.
		"""
		_trans = theta0 + 90  # shift value 90 degrees from N->E

		if _trans > -180 and _trans < 0:
			_trans = 360 + _trans  # transform angle to 0->360 if in -180->0 quadrants

		return _trans


	def determine_drive_distance(self, A, B):
		return math.sqrt((B[1] - A[1])**2 + (B[0] - A[0])**2)


	def quat_to_angle(self, quat):
		"""
		Converts quaternion to angle.
		"""
		rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
		return rot.GetRPY()[2]


	def get_odom(self):
		# Get the current transform between the odom and base frames
		try:
			(trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
		except (tf.Exception, tf.ConnectivityException, tf.LookupException):
			rospy.loginfo("TF Exception")
			return

		return (Point(*trans), self.quat_to_angle(Quaternion(*rot)))


	def shutdown(self):
		"""
		Always stop the robot when shutting down the node
		"""
		rospy.loginfo(">>>>> Stopping the robot by publishing blank Twist to jackal_nav_controller..")
		# jackal_nav_controller.cmd_vel.publish(Twist())  # stop that robot!
		self.nav_controller.cmd_vel.publish(Twist())
		rospy.sleep(1)


	def get_jackal_pos_from_service(self):
		"""
		Get current GPS fix from Jackal's position
		"""
		rospy.wait_for_service('get_jackal_pos')
		get_jackal_pos = rospy.ServiceProxy('get_jackal_pos', JackalPos)
		return get_jackal_pos()



	# def flag_callback(self, at_flag):
	# 	"""
	# 	Subscribes to /at_flag topic that's being published by
	# 	jackal_flag_node.py. Needs to stop Jackal if at_flag is True
	# 	"""
	# 	print("At flag?: {}".format(at_flag.data))

	# 	if at_flag.data == True or at_flag == True:
	# 		print("Shutting down Jackal cause we're at the flag!!!")
	# 		self.at_flag = True  # sets main at_flag to True for robot..
	# 		# self.shutdown()
	# 		# return





if __name__ == '__main__':

	try:
		course_filename = sys.argv[1]
	except IndexError:
		raise IndexError("Course not specified. Add course filename as arg when running basic_drive_5.py")

	coursefile = open(course_filename, 'r')
	course = json.loads(coursefile.read())

	try:
		SingleGoalNav(course)
		# SingleGoalNav()
	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation terminated.")
		rospy.loginfo("Shutting down drive node!")
		raise Exception("basic drive ROS node exception")

	rospy.spin()