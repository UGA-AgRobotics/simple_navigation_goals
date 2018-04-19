#!/usr/bin/env python

"""
Use jackal_pos_server and jackal_rot_server to do some simple goal-oriented driving.

Basically a modified version of the current jackal_pos_client.py. The goal is
to drive from A --> B by first rotation toward B, then driving to the point.

This is the same as basic_drive_2.py except it's going to correct itself after
a given step size instead of doing a one-and-done calculation from A->B
"""



import roslib
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from simple_navigation_goals.srv import *
import tf
# from transform_utils import quat_to_angle, normalize_angle
import math
from math import radians, copysign, sqrt, pow, pi, degrees
import utm
import PyKDL
import numpy as np
import matplotlib.pyplot as plt
import dubins

from nav_tracks import NavTracks



class SingleGoalNav():
	"""
	Testing Jackal navigation to a single goal/flag. Determines
	X and Y distance to travel using its GPS location and flag's 
	location, both converted to UTM.
	"""

	def __init__(self):
		# Give the node a name
		rospy.init_node('single_goal_nav', anonymous=False)
		
		# Set rospy to exectute a shutdown function when terminating the script
		rospy.on_shutdown(self.shutdown)

		# How fast will we check the odometry values?
		rate = 20
		
		# Set the equivalent ROS rate variable
		self.r = rospy.Rate(rate)
		
		# Set the parameters for the target square
		goal_distance = rospy.get_param("~goal_distance", 1.0)      # meters
		goal_angle = rospy.get_param("~goal_angle", radians(90))    # degrees converted to radians
		linear_speed = rospy.get_param("~linear_speed", 0.2)        # meters per second
		angular_speed = rospy.get_param("~angular_speed", 0.7)      # radians per second
		angular_tolerance = rospy.get_param("~angular_tolerance", radians(2)) # degrees to radians



		self.step_size = 0.5  # step size to break up A->B distances (in meters)
		self.time_step_size = self.step_size / linear_speed


		
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
			self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
			self.base_frame = '/base_footprint'
		except (tf.Exception, tf.ConnectivityException, tf.LookupException):
			try:
				self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
				self.base_frame = '/base_link'
			except (tf.Exception, tf.ConnectivityException, tf.LookupException):
				rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
				rospy.signal_shutdown("tf Exception")



		position = Point()  # initialize the position variable as a Point type
		move_cmd = Twist()  # initialize movement comment

		move_cmd.linear.x = linear_speed  # set movement command to forward motion

		(position, rotation) = self.get_odom()  # get starting position values

		# Test 2: Attempt at single goal navigation to flag..
		# _track = NavTracks().get_track('track4')
		_track = NavTracks().get_track('track4')  # just one initial goal point for initial dubins testing..

		_np_track = np.array(_track)



		qs_array = []
		turning_radius = 1.0
		# step_size = 0.5
		step_size = 0.2


		############ TESTING A SINGLE A->B ################################
		curr_pose = self.call_jackal_pos_service(0)  # don't drive, just get current lat/lon

		print("Current position from pose server: {}".format(curr_pose))

		_lat = curr_pose.jackal_fix.latitude
		_lon = curr_pose.jackal_fix.longitude

		print("Jackal's current lat, lon: {}, {}".format(_lat, _lon))

		curr_pose_utm = utm.from_latlon(curr_pose.jackal_fix.latitude, curr_pose.jackal_fix.longitude)

		# A = (curr_pose_utm[0], curr_pose_utm[1], rotation)  # initial position of jackal
		A = (curr_pose_utm[0], curr_pose_utm[1], rotation)  # initial position of jackal
		B = (_track[0][0], _track[0][1], math.pi)  # use calculated orientation above

		print("Jackal's current angle in degrees: {}".format(math.degrees(A[2])))

		x_diff = B[0] - A[0]
		y_diff = B[1] - A[1]


		AB_theta0 = math.atan2(abs(y_diff), abs(x_diff))  # get intitial angle, pre transform
		AB_angle = self.transform_angle_by_quadrant(AB_theta0, x_diff, y_diff)  # determine angle between vector A and B

		print("initial angle: {}".format(math.degrees(AB_theta0)))
		print("AB angle: {}".format(AB_angle))

		# A = (curr_pose_utm[0], curr_pose_utm[1], math.radians(AB_angle))

		# trying just the single imu transform function:
		transformed_angle = self.transform_imu_frame(math.degrees(rotation))
		print("Transform angle 1 : {}".format(transformed_angle))
		# transformed_angle = self.initiate_angle_transform(A, B)
		# print("Transform angle 2 : {}".format(transformed_angle))
		# transformed_angle = self.transform_angle_by_quadrant(rotation, x_diff, y_diff)
		# print("Transform angle 3 : {}".format(transformed_angle))
		A = (curr_pose_utm[0], curr_pose_utm[1], math.radians(transformed_angle))
		# A = (curr_pose_utm[0], curr_pose_utm[1], rotation + math.pi/4)
		# print(">>> A with transformed angle: {}".format(A))
		# print("Jackal's goal orientation: {}".format(degrees(goal_orientation)))
		print("Jackal's transformed angle in degrees: {}".format(math.degrees(A[2])))

		qs,_ = dubins.path_sample(A, B, turning_radius, step_size)
		qs = np.array(qs)

		dubins_data = {
			'q0': A,
			'q1': B,
			'qs': qs
		}
		qs_array.append(dubins_data)

		# self.plot_full_dubins_path([dubins_data], _np_track[:,0], _np_track[:,1])

		for pos_tuple in qs:
			self.p2p_drive_routine(pos_tuple)  # will do calc-turn-calc-drive as request-response

		#############################################################################################





		# # Looping track points here (items of [easting, northing]):
		# # for goal_pos in _track:
		# for _track_int in range(0, len(_track) - 1):

		# 	(position, rotation) = self.get_odom()  # get starting position values

		# 	goal_pos = _track[_track_int]
		# 	future_goal_pos = _track[_track_int + 1]  # using future goal pos to determine orientation at current goal pos
		# 	x_diff = future_goal_pos[0] - goal_pos[0]
		# 	y_diff = future_goal_pos[1] - goal_pos[1]
		# 	goal_orientation = math.atan2(abs(y_diff), abs(x_diff))  # get intitial angle, pre transform

		# 	print(">>> Initial goal orientation (degrees): {}".format(math.degrees(goal_orientation)))

		# 	# Assuming Jackal is somewhat facing point (I or II quadrant):
		# 	if x_diff > 0:
		# 		# turn right (relative to north)
		# 		pass
		# 	elif x_diff < 0:
		# 		# turn left (relative to north)
		# 		goal_orientation = math.pi + goal_orientation

		# 	curr_pose = self.call_jackal_pos_service(0)  # don't drive, just get current lat/lon

		# 	print("Current position from pose server: {}".format(curr_pose))

		# 	_lat = curr_pose.jackal_fix.latitude
		# 	_lon = curr_pose.jackal_fix.longitude

		# 	print("Jackal's current lat, lon: {}, {}".format(_lat, _lon))

		# 	curr_pose_utm = utm.from_latlon(curr_pose.jackal_fix.latitude, curr_pose.jackal_fix.longitude)

		# 	# A = (curr_pose_utm[0], curr_pose_utm[1], rotation)  # initial position of jackal
		# 	A = (curr_pose_utm[0], curr_pose_utm[1], rotation)  # initial position of jackal
		# 	B = (goal_pos[0], goal_pos[1], goal_orientation)  # use calculated orientation above

		# 	print("Jackal's current angle in degrees: {}".format(math.degrees(A[2])))


		# 	# transformed_angle = self.initiate_angle_transform(A, B)  # in degrees
		# 	# print(">>> Transform angle in degrees: {}".format(transformed_angle))

		# 	# x_diff = B[0] - A[0]
		# 	# y_diff = B[1] - A[1]
		# 	# AB_theta0 = math.atan2(abs(y_diff), abs(x_diff))  # get intitial angle, pre transform
		# 	# AB_angle = self.transform_angle_by_quadrant(AB_theta0, x_diff, y_diff)  # determine angle between vector A and B
		# 	# # current imu angle transformed into jackal's frame:
		# 	# print("Transformed angle, pre turning considerations: {}".format(AB_angle))


		# 	# trying just the single imu transform function:
		# 	transformed_angle = self.transform_imu_frame(math.degrees(rotation))

		# 	print("Angle after single IMU transform: {}".format(transformed_angle))

		# 	# print(">>> Original A: {}".format(A))
		# 	# # reset A to have transformed angle?
		# 	A = (curr_pose_utm[0], curr_pose_utm[1], math.radians(transformed_angle))
		# 	print(">>> A with transformed angle: {}".format(A))


		# 	# print("Jackal's position in UTM: {}".format(A))
		# 	# print("Goal pos: {}".format(goal_pos))
		# 	# print("Jackal's goal position in UTM: {}".format(B))
		# 	print("Jackal's goal orientation: {}".format(degrees(goal_orientation)))

		# 	qs,_ = dubins.path_sample(A, B, turning_radius, step_size)
		# 	qs = np.array(qs)

		# 	dubins_data = {
		# 		'q0': A,
		# 		'q1': B,
		# 		'qs': qs
		# 	}
		# 	qs_array.append(dubins_data)


		# 	# self.plot_dubins_path(dubins_path, A, B, show=True)

		# 	# print("Plotting dubins path from A to B..")
		# 	# self.plot_full_dubins_path([dubins_data], _np_track[:,0], _np_track[:,1])

		# 	# for pos_tuple in qs:
		# 	# 	# loop each x,y,orientation tuple from dubins model..
		# 	# 	self.p2p_drive_routine(pos_tuple)  # will do calc-turn-calc-drive as request-response


		print("Shutting down Jackal..")
		self.shutdown()



	def p2p_drive_routine(self, goal_pos):
		"""
		The drive routine from point-to-point, whether that's b/w
		two GPS points on the course, or a step size incrementing a drive
		between two GPS points.
		"""
		(position, rotation) = self.get_odom()  # get starting position values

		curr_pose = self.call_jackal_pos_service(0)  # don't drive, just get current lat/lon

		print("Current position from pose server: {}".format(curr_pose))

		_lat = curr_pose.jackal_fix.latitude
		_lon = curr_pose.jackal_fix.longitude

		print("Jackal's current lat, lon: {}, {}".format(_lat, _lon))

		curr_pose_utm = utm.from_latlon(curr_pose.jackal_fix.latitude, curr_pose.jackal_fix.longitude)

		print("Jackal's position in UTM: {}".format(curr_pose_utm))

		A = (curr_pose_utm[0], curr_pose_utm[1], rotation)
		B = (goal_pos[0], goal_pos[1], goal_pos[2])
		
		turn_angle = self.initiate_angle_transform(A, B)

		if turn_angle != 0:
			# Determine angle to turn based on IMU..
			print("Telling Jackal to turn {} degreess..".format(turn_angle))
			self.call_jackal_rot_service(turn_angle)
			print("Finished turning..")

		drive_distance = self.determine_drive_distance(A, B)

		if drive_distance > 0:
			print("Driving Jackal {} meters..".format(drive_distance))
			self.call_jackal_pos_service(drive_distance)
			print("Finished driving..")



	def plot_full_dubins_path(self, qs_array, x_path=None, y_path=None):
		"""
		Like plot_dubins_path() function, but plots a full set of points
		instead a single A -> B two point dataset.
		"""
		# Initial setup: No directional plotting, just dots and path at the moment..

		for qs in qs_array:
			self.plot_dubins_path(qs['qs'], qs['q0'], qs['q1'], show=False)

		plt.plot(x_path, y_path, 'bo')  # overlay path points onto plot

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

		print("QS Array: {}".format(qs))

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
		# plt.plot(sample_points[:,0], sample_points[:,1], 'k-')  # plot sample points path
		for i in xrange(qs.shape[0]):
			plt.plot([xs[i], us[i]], [ys[i], vs[i]],'r-')

		# Adding x/y range for plots:
		# plt.xlim(min(qs[:,0]) - 1, max(qs[:,0]) + 1)
		# plt.ylim(min(qs[:,1]) - 1, max(qs[:,1]) + 1)

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

		print("A: {}".format(A))
		print("B: {}".format(B))
		print("x_diff: {}".format(x_diff))
		print("y_diff: {}".format(y_diff))

		_trans_angle = self.transform_imu_frame(degrees(A[2]))
		AB_theta0 = math.atan2(abs(y_diff), abs(x_diff))  # get intitial angle, pre transform
		AB_angle = self.transform_angle_by_quadrant(AB_theta0, x_diff, y_diff)  # determine angle between vector A and B

		turn_angle = None
		if AB_angle == 0:
			turn_angle = 0
		else:
			turn_angle = AB_angle - _trans_angle  # angle to turn (signage should denote direction to turn)

		print("Initial position and orientation: {}".format(A))
		print("Current angle in degrees: {}".format(degrees(A[2])))
		print("Transformed angle: {}".format(_trans_angle))
		print("AB initial angle: {}".format(degrees(AB_theta0)))
		print("AB angle after transform: {}".format(AB_angle))
		print("Calculated turning angle: {}".format(turn_angle))

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
			raise "!Error occurred in basic_drive_3/transform_angle_by_quadrant func.."



	def get_angle_between_vectors(self, A, B):
		"""
		Uses the dot product to determine the angle
		between points A and B.

		cos(theta) = (A dot B) / (||A|| + ||B||)
		"""
		dot_prod = A[0]*B[0] + A[1]*B[1]
		len_A = math.sqrt(A[0]**2 + A[1]**2)
		len_B = math.sqrt(B[0]**2 + B[1]**2)

		return math.acos(dot_prod / (len_A + len_B))



	def transform_imu_frame(self, theta0):
		"""
		Transform initial robot angle in IMU frame to have 0 degrees
		in East direction, going 0->360 CCW.
		"""
		_trans = theta0 + 90  # shift value 90 degrees from N->E

		if _trans > -180 and _trans < 0:
			_trans = 360 + _trans  # transform angle to 0->360 if in -180->0 quadrants

		return _trans


	def determine_turn_angle(self, A, B):
		curr_angle = degrees(A[2])
		ab_angle = degrees(math.atan2((B[1] - A[1]), (B[0] - A[0])))  # determine angle between A and B in world/UTM frame

		print("Current angle of robot: {}".format(curr_angle))
		print("Angle between A and B: {}".format(ab_angle))

		if curr_angle > 0:
			return 180 - (curr_angle - ab_angle)

		if curr_angle < 0:
			return -180 - (curr_angle - ab_angle)


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
		rospy.loginfo("Stopping the robot...")
		self.cmd_vel.publish(Twist())
		rospy.sleep(1)


	def convert_to_utm(self, lat, lon):
		"""
		Convert lat/lon to utm.
		"""
		try:
			return utm.from_latlon(lat, lon)
		except e as Exception:
			print("Error converting lat/lon to utm: {}".format(e))
			return None


	def call_jackal_pos_service(self, distance):
		"""
		Get current GPS fix from Jackal's position
		"""
		rospy.wait_for_service('get_jackal_pos')
		get_jackal_pos = rospy.ServiceProxy('get_jackal_pos', JackalPos)
		return get_jackal_pos(distance)


	def call_jackal_rot_service(self, angle):
		"""
		Get current IMU position and orientation from Jackal.
		Inputs:
			angle - angle to turn in radians
		"""
		rospy.wait_for_service('get_jackal_rot')
		get_jackal_rot = rospy.ServiceProxy('get_jackal_rot', JackalRot)
		return get_jackal_rot(angle)



if __name__ == '__main__':
	try:
		SingleGoalNav()
	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation terminated.")