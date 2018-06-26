#!/usr/bin/env python

"""
The idea for this node is to keep track of the current position of
the robot and the path/course it's following as well. When the robot is
within some look-ahead of the goal point in the path, then the angle to
turn toward the next point needs to be calculated and published
to the turning node.
"""

import roslib
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from simple_navigation_goals.srv import *
import tf
from math import radians, copysign, sqrt, pow, pi, degrees
import PyKDL
import utm
from nav_tracks import NavTracks
import orientation_transforms
import json
import numpy as np



# Global settings:
tf_listener = tf.TransformListener()
odom_frame = '/odom'
base_frame = '/base_link'
# cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)  # see http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers#Choosing_a_good_queue_size
linear_speed = 0.4  # units of m/s
rate = 20  # Hz
angular_speed = 0.4
angular_tolerance = rospy.get_param("~angular_tolerance", radians(2)) # degrees to radians




class NavNode:

	def __init__(self, path=None):

		print("Starting nav_node.py..")
		rospy.init_node('nav_node', anonymous=True)

		# Subscribers
		# rospy.Subscriber("/at_flag", Bool, self.flag_callback)  # sub to /at_flag topic from jackal_flags_node.py
		# rospy.Subscriber("/rf_stop", Bool, self.rf_stop_callback, queue_size=1)
		# rospy.Subscriber("/fix", NavSatFix, self.position_callback, queue_size=1)
		# rospy.Subscriber("sample_collected", Bool, self.sample_collected_callback)

		# Publishers:
		self.turn_pub = rospy.Publisher('/rover_turn', Float64, queue_size=1)
		self.drive_pub = rospy.Publisher('/start_driving', Bool, queue_size=1)
		self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)  # see http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers#Choosing_a_good_queue_size
		# self.sample_publisher = rospy.Publisher('collect_sample', Bool, queue_size=1)

		# rospy.wait_for_service('start_sample_collection')
		# self.start_sample_collection = rospy.ServiceProxy('start_sample_collection', SampleCollection)

		print("Waiting for get_jackal_pos service..")
		rospy.wait_for_service('get_jackal_pos')
		self.get_jackal_pos = rospy.ServiceProxy('get_jackal_pos', JackalPos)
		print("get_jackal_pos service ready.")

		print("Waiting for get_jackal_rot service..")
		rospy.wait_for_service('get_jackal_rot')
		self.get_jackal_rot = rospy.ServiceProxy('get_jackal_rot', JackalRot)
		print("get_jackal_rot service ready.")

		self.nav_tracks = NavTracks()

		if not path:
			raise Exception("Must provide path as argument to nav_node.py..")

		self.path = path

		self.move_cmd = Twist()
		self.look_ahead = 1.0

		self.at_flag = False
		self.emergency_stop = False

		self.current_utm = [None, None]  # current easting, northing position
		self.current_goal = None

		print("Initiating driving, publishing True to drive_node.py /start_driving topic")
		# rospy.sleep(1)
		self.drive_pub.publish(True)

		print("Starting navigation..")
		self.start_navigation()

		print("nav_node ready.")

		rospy.spin()



	def start_navigation(self):
		"""
		Main node loop.
		"""
		_np_path = self.set_path_to_follow()  # returns numpy array of current path..

		# Initial GPS position:
		curr_pose_utm = self.get_current_position()

		# Accounting for look-ahead distance:
		target_index = self.calc_target_index(curr_pose_utm, 0, _np_path[:,0], _np_path[:,1])
		print("Initial target goal index: {}".format(target_index))

		# Loop track goals here for each A->B in the course:
		for i in range(target_index, len(self.path) - 1):

			current_goal = self.path[i]  # current goal in the path
			future_goal = None
			try:
				# future_goal = _track[i + 1]
				future_goal = self.path[i + 1]
			except IndexError as e:
				print("Finished course goals! Shutdown ROS? Stop rover?")
				break

			curr_pose_utm = self.get_current_position()
			(position, rotation) = self.get_odom()  # get starting position values
			# curr_angle = self.quat_to_angle(self.get_jackal_rot().orientation)
			# print("Current angle from get_jackal_rot: {}".format(curr_angle))

			goal_orientation = orientation_transforms.determine_angle_at_goal(current_goal, future_goal)  # angle at goal position
			# transformed_angle = orientation_transforms.transform_imu_frame(degrees(curr_angle))  # current angle
			transformed_angle = orientation_transforms.transform_imu_frame(degrees(rotation))

			A = (curr_pose_utm[0], curr_pose_utm[1], radians(transformed_angle))  # initial position of jackal
			B = (current_goal[0], current_goal[1], goal_orientation)

			drive_distance = self.determine_drive_distance(A, B)  # get drive distance from current position to goal

			# Skips to next goal/course point if said goal is less than look-ahead:
			if drive_distance < self.look_ahead:
				print("Within look-ahead of goal, so skipping to next goal")
				continue  # skip to next iteration in track for loop

			# Sets current goal:
			self.current_goal = current_goal  # note: current_goal is set if greater than look-ahead from rover..

			self.p2p_continuous_drive(self.current_goal)  # initiates driving loop..


		print("Finished driving routine! Stopping rover..")
		self.cmd_vel.publish(Twist())
		self.drive_pub.publish(False)  # tell drive_node.py to stop
		print("Rover stopped.")



	def p2p_continuous_drive(self, current_goal):

		curr_pose_utm = self.get_current_position()  # Gets updated position of rover..
		(position, rotation) = self.get_odom()  # get starting position values
		
		A = (curr_pose_utm[0], curr_pose_utm[1], rotation)
		# B = (goal_pos[0], goal_pos[1], rotation)  # NOTE: B's orientation currently hardcoded for testing..
		B = (current_goal[0], current_goal[1], rotation)
		
		turn_angle = orientation_transforms.initiate_angle_transform(A, B)

		if turn_angle != 0:
			# Determine angle to turn based on IMU..
			print("Telling Jackal to turn {} degreess..".format(turn_angle))
			# self.nav_controller.execute_turn(radians(turn_angle))
			self.turn_pub.publish(radians(turn_angle))


		print("Current goal: {}".format(current_goal))
		print("About to start distance from goal loop...")



		distance_from_goal = sqrt(pow((self.current_goal[0] - curr_pose_utm[0]), 2) +
								pow((self.current_goal[1] - curr_pose_utm[1]), 2))



		# distance_from_goal = 0
		# while distance_from_goal < self.look_ahead and not rospy.is_shutdown():
		while distance_from_goal > self.look_ahead and not rospy.is_shutdown():
			# Loops until rover is within look ahead distance of current goal, then moves on to next goal in path:

			curr_pose_utm = self.get_current_position()
			# (position, rotation) = self.get_odom()  # get starting position values

			# distance_from_goal = sqrt(pow((self.current_goal[0] - self.current_utm[0]), 2) +
			# 								pow((self.current_goal[1] - self.current_utm[1]), 2))
			distance_from_goal = sqrt(pow((self.current_goal[0] - curr_pose_utm[0]), 2) +
									pow((self.current_goal[1] - curr_pose_utm[1]), 2))

			print("Distance from goal: {}".format(distance_from_goal))

			rospy.sleep(1.0/rate)


		print(">>> Within look-ahead, moving to next goal..")
		return



	def set_path_to_follow(self):
		"""
		Sets path attribute, where a path consists of a list
		of [easting, northing] pairs.
		"""

		if not self.path:
			raise Exception("Must specify path")

		_path = self.nav_tracks.get_track_from_course(self.path)  # builds track from a GPS course/path
		_np_path = np.array(_path)

		print("The Course: {}".format(_path))

		self.path = _path  # sets path as list of [easting, northing] pairs

		return _np_path  # returns numpy array of track for easy array manipulation



	def get_current_position(self):
		# curr_pose = self.get_jackal_pos_from_service()  # don't drive, just get current lat/lon
		curr_pose = self.get_jackal_pos()
		curr_pose_utm = utm.from_latlon(curr_pose.jackal_fix.latitude, curr_pose.jackal_fix.longitude)
		return curr_pose_utm



	def calc_target_index(self, current_position, current_goal_index, cx, cy):
		"""
		From red_rover_model pure_puruit module. Loops through course
		points (x and y) and builds a list of the diff b/w robot's position and
		each x and y in the course. Finally, 
		"""
		dx = [current_position[0] - icx for icx in cx[current_goal_index:]]  # diff b/w robot's position and all x values in course (starting at current goal, onward)
		dy = [current_position[1] - icy for icy in cy[current_goal_index:]]  # diff b/w robot's position and all y values in course (starting at current goal, onward)

		d = [abs(sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]  # scalar diff b/w robot and course values

		print("Determining goal point based on look-ahead of {}".format(self.look_ahead))

		ind = 0
		for pos_diff in d:
			# print("Distance between Jackal and goal: index={}, value=({}, {}), distance={} meters".format(ind, cx[ind], cy[ind], d[ind]))
			print("Distance between Jackal and goal: index={}, value=({}, {}), distance={} meters".format(ind, cx[ind], cy[ind], pos_diff))
			if pos_diff > self.look_ahead:
				return d.index(pos_diff)  # return index of goal to go to
			ind += 1



	# def flag_callback(self, flag_msg):
	# 	"""
	# 	Subscribes to /at_flag topic that's being published by
	# 	jackal_flag_node.py. Needs to stop Jackal if at_flag is True
	# 	"""
	# 	# print("At flag? {}".format(flag_msg.data))

	# 	if flag_msg.data == True or flag_msg == True:
	# 		print("Stopping cause we're at the flag!!!")
	# 		self.at_flag = True  # sets main at_flag to True for robot..



	# def rf_stop_callback(self, stop_msg):
	# 	"""
	# 	/rf_stop is an emergency stop from the arduino, which uses a 
	# 	32197-MI 4 Ch. remote control receiver
	# 	"""
	# 	print("Received RF stop message! {}".format(stop_msg))
	# 	if stop_msg.data == True:
	# 		self.emergency_stop = True
	# 	else:
	# 		self.emergency_stop = False



	# def goal_pos_callback(self, goal_msg):
	# 	"""
	# 	Subscribes to /current_goal topic that's the current position in the path
	# 	the robot is driving toward.
	# 	"""
	# 	pass




	# def position_callback(self, pos_msg):
	# 	"""
	# 	GPS position from Emlid unit at /fix topic.
	# 	"""
	# 	# Compare the current position with the path goal..

	# 	current_utm = utm.from_latlon(pos_msg.latitude, pos_msg.longitude)

	# 	# self.current_utm = []
	# 	self.current_utm = [current_utm[0], current_utm[1]]
	# 	print("Position callback, current UTM: {}".format(self.current_utm))




	def get_odom(self):
		# Get the current transform between the odom and base frames
		try:
			(trans, rot)  = tf_listener.lookupTransform(odom_frame, base_frame, rospy.Time(0))
		except (tf.Exception, tf.ConnectivityException, tf.LookupException):
			rospy.loginfo("TF Exception")
			return

		return (Point(*trans), self.quat_to_angle(Quaternion(*rot)))



	def quat_to_angle(self, quat):
		"""
		Converts quaternion to angle.
		"""
		rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
		return rot.GetRPY()[2]



	def normalize_angle(self, angle):
		res = angle
		while res > pi:
			res -= 2.0 * pi
		while res < -pi:
			res += 2.0 * pi
		return res



	def determine_drive_distance(self, A, B):
		return sqrt((B[1] - A[1])**2 + (B[0] - A[0])**2)







if __name__ == '__main__':


	try:
		path_filename = sys.argv[1]
	except IndexError:
		raise IndexError("Course not specified. Add course filename as arg when running basic_drive_5.py")

	pathfile = open(path_filename, 'r')
	path = json.loads(pathfile.read())

	try:
		NavNode(path)
	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation terminated.")
		rospy.loginfo("Shutting down drive node!")
		raise Exception("basic drive ROS node exception")