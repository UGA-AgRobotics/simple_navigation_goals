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



		position = Point()  # initialize the position variable as a Point type
		move_cmd = Twist()  # initialize movement comment

		move_cmd.linear.x = linear_speed  # set movement command to forward motion

		(position, rotation) = self.get_odom()  # get starting position values

		# Test 2: Attempt at single goal navigation to flag..
		# _track = NavTracks().get_track('track4')
		_track = NavTracks().get_track('track4')  # just one initial goal point for initial dubins testing..

		_np_track = np.array(_track)



		# Main model constants (todo: move main model constants to top):
		######################################################################
		qs_array = []  # collection of points for dubins paths
		turning_radius = 2.5  # min turning radius for robot in meters
		step_size = 0.5  # dubins model step size in meters
		look_ahead = 2.0  # look-ahead distance in meters
		######################################################################


		# # # Sleep routine for testing:
		# print("Pausing 10 seconds before initiating driving (to have time to run out there)...")
		# rospy.sleep(10)
		# print("Initiating driving to point B..")



		# Initial GPS position:
		curr_pose_utm = self.get_current_position()

		# Accounting for look-ahead distance:
		target_index = self.calc_target_index(curr_pose_utm, 0, _np_track[:,0], _np_track[:,1], look_ahead)
		print("Target goal index: {}".format(target_index))


		# Loop track goals here for each A->B in the course:
		for i in range(target_index, len(_track) - 1):

			current_goal = _track[i]
			future_goal = None

			try:
				future_goal = _track[i + 1]
			except IndexError as e:
				print("Current goal is the last one in the course!")
				print("End at the same orientation as the last goal..")
				pass  # continue on..

			goal_orientation = self.determine_angle_at_goal(current_goal, future_goal)
			print("Goal orientation when robot arrives at B: {}".format(math.degrees(goal_orientation)))



			############ TESTING A SINGLE A->B ################################

			curr_pose = self.call_jackal_pos_service(0)  # don't drive, just get current lat/lon

			# print("Current position from pose server: {}".format(curr_pose))

			_lat = curr_pose.jackal_fix.latitude
			_lon = curr_pose.jackal_fix.longitude

			print("Jackal's current lat, lon: {}, {}".format(_lat, _lon))

			curr_pose_utm = utm.from_latlon(curr_pose.jackal_fix.latitude, curr_pose.jackal_fix.longitude)

			transformed_angle = rotation
			# print("Initial angle from Jackal IMU: {}".format(degrees(transformed_angle)))
			transformed_angle = self.transform_imu_frame(degrees(transformed_angle))
			# print("Angle after IMU transform: {}".format(transformed_angle))  # i think this is the appropriate transformed angle

			A = (curr_pose_utm[0], curr_pose_utm[1], math.radians(transformed_angle))  # initial position of jackal
			B = (current_goal[0], current_goal[1], goal_orientation)

			print("Jackal's current angle in degrees: {}".format(math.degrees(A[2])))

			x_diff = B[0] - A[0]
			y_diff = B[1] - A[1]

			AB_theta0 = math.atan2(abs(y_diff), abs(x_diff))  # get intitial angle, pre transform
			AB_angle = self.transform_angle_by_quadrant(AB_theta0, x_diff, y_diff)  # determine angle between vector A and B

			qs,_ = dubins.path_sample(A, B, turning_radius, step_size)
			qs = np.array(qs)

			dubins_data = {
				'q0': A,
				'q1': B,
				'qs': qs
			}
			qs_array.append(dubins_data)

			goals_array = self.build_goals_from_dubins(qs)  # NOTE: This includes the position the Jackal is currently, so skip first goal

			print("Jackal's current UTM position: {}".format(curr_pose_utm))
			print("Goals array for Jackal to follow: {}".format(goals_array))
			print("Number of points to Goal: {}".format(len(goals_array)))

			# self.plot_full_dubins_path([dubins_data], _np_track[:,0], _np_track[:,1], show=False, filename="dubinsAB_{}".format(i))  # saves plotsas model runs
			self.plot_full_dubins_path([dubins_data], _np_track[:,0], _np_track[:,1])  # shows plot mid model (blocks model for each plot)


			# # Pause routine for testing (walking b/w lab and field)
			# print("Pausing 10 seconds before initiating driving (to have time to run out there)...")
			# rospy.sleep(10)
			# print("Initiating driving to point B..")




			# # NOTE: current goal index = i, future goal = i + 1;
			# # If target_index > current goal index, start going to next goal..
			# # Drive routine to follow the steps from A->B dubins, and checks for look-ahead to path goals (not dubins points)..
			# for goal in goals_array[1:]:

			# 	print("Executing drive routine..")
			# 	self.p2p_drive_routine(goal)

			# 	curr_pose_utm = self.get_current_position()
			# 	print("Jackal's current position in UTM: {}".format(curr_pose_utm))

			# 	print("Finished driving, now determining target index..")
			# 	new_target_index = self.calc_target_index(curr_pose_utm, target_index, _np_track[:,0], _np_track[:,1], look_ahead)

			# 	if new_target_index > target_index:
			# 		print(">>> Jackal is within look-ahead distance of goal, starting to drive toward next goal now..")
			# 		target_index = new_target_index
			# 		break  # break out of drive loop and start driving toward next goal in track!


		print("Shutting down Jackal..")
		self.shutdown()



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
			self.call_jackal_rot_service(turn_angle)
			print("Finished turning..")

		drive_distance = self.determine_drive_distance(A, B)

		if drive_distance > 0:
			print("Driving Jackal {} meters..".format(drive_distance))
			self.call_jackal_pos_service(drive_distance)
			print("Finished driving..")



	def calc_target_index(self, current_position, current_goal_index, cx, cy, look_ahead):
		"""
		From red_rover_model pure_puruit module. Loops through course
		points (x and y) and builds a list of the diff b/w robot's position and
		each x and y in the course. Finally, 
		"""
		dx = [current_position[0] - icx for icx in cx[current_goal_index:]]  # diff b/w robot's position and all x values in course (starting at current goal, onward)
		dy = [current_position[1] - icy for icy in cy[current_goal_index:]]  # diff b/w robot's position and all y values in course (starting at current goal, onward)

		d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]  # scalar diff b/w robot and course values

		print("Determining goal point based on look-ahead of {}".format(look_ahead))

		ind = 0
		for pos_diff in d:
			print("Distance between Jackal and goal: index={}, value=({}, {}), distance={} meters".format(ind, cx[ind], cy[ind], d[ind]))
			if pos_diff > look_ahead:
				return d.index(pos_diff)  # return index of goal to go to
			ind += 1


		# L = 0.0
		# while look_ahead > L and (ind + 1) < len(cx):
		# 	dx = cx[ind + 1] - cx[ind]
		# 	dy = cy[ind + 1] - cy[ind]
		# 	L += math.sqrt(dx ** 2 + dy ** 2)
		# 	print("Distance b/w points: {}".format(L))
		# 	ind += 1

		# print("Target index: {}".format(ind))
		# print("Target goal after look-ahead: {}, {}".format(cx[ind], cy[ind]))

		# return ind



	def get_current_position(self):

		curr_pose = self.call_jackal_pos_service(0)  # don't drive, just get current lat/lon

		_lat = curr_pose.jackal_fix.latitude
		_lon = curr_pose.jackal_fix.longitude

		print("Jackal's current lat, lon: {}, {}".format(_lat, _lon))

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