#!/usr/bin/env python

"""
This is the Red Rover version of jackal_drive_test.py.
It'll follow rows with one set of parameters, then follow
a curved path to the next row with the dubins model, changing the
drive paramters for the curved path.
"""

import roslib
import rospy
from std_msgs.msg import Bool, Float64, UInt8, Int64
from sensor_msgs.msg import NavSatFix, Imu
from mico_leaf_msgs.srv import start_sample
import sys
import math
import json
import utm
from math import radians, copysign, sqrt, pow, pi, degrees
import PyKDL
import numpy as np
from geometry_msgs.msg import Quaternion, Twist

# Local package requirements:
from lib.nav_tracks import NavTracks
from lib.nav_nudge import NavNudge
from lib import orientation_transforms
from lib import dubins_path as dp



class RedRoverDriveDubins(object):
	"""
	Testing Rover navigation.
	Drives based on rover's position, a look-ahead goal in a course,
	and its orientatiion. Subscribes to GPS and IMU topics. This routine
	follows a multi-row course.
	"""

	def __init__(self, path_json, nudge_factor=None):
		
		# Give the node a name
		rospy.init_node('red_rover_drive_dubins')

		print("Starting red rover driver node..")

		# Subscribers:
		rospy.Subscriber("/start_driving", Bool, self.start_driving_callback_multirow, queue_size=1)
		rospy.Subscriber("/fix", NavSatFix, self.rover_position_callback, queue_size=1)
		rospy.Subscriber('/phidget/imu/data', Imu, self.rover_imu_callback, queue_size=1)
		rospy.Subscriber("/at_flag", Bool, self.flag_callback)  # sub to /at_flag topic from jackal_flags_node.py
		rospy.Subscriber("/flag_index", Int64, self.flag_index_callback)

		# Publishers:
		self.actuator_pub = rospy.Publisher('/driver/linear_drive_actuator', Float64, queue_size=1)  # TODO: double check queue sizes..
		self.throttle_pub = rospy.Publisher('/driver/throttle', UInt8, queue_size=1)  # TODO: double check queue sizes..
		self.articulator_pub = rospy.Publisher('/driver/articulation_relay', Float64, queue_size=1)  # TODO: double check queue sizes..


		# Set rospy to exectute a shutdown function when terminating the script
		rospy.on_shutdown(self.shutdown)

		# How fast will we check the odometry values?
		self.rate = 10
		
		# Set the equivalent ROS rate variable
		self.r = rospy.Rate(self.rate)

		self.path_json = path_json  # The path/course the red rover will follow!



		# TODO: HAVE THIS NUDGE FEATURE WITHIN DRIVE ROUTINE TO NUDGE ONLY STRAIGHT ROWS:
		if nudge_factor and isinstance(nudge_factor, float):
			print("Using nudge factor of {} to shift the course!".format(nudge_factor))
			nn = NavNudge(json.dumps(path_json), nudge_factor, 0.2)  # NOTE: HARD-CODED SPACING FACTOR TO 0.2M FOR NOW
			self.path_json = nn.nudged_course



		self.path_array = None  # path converted to list of [easting, northing]

		self.angle_tolerance = 0.1  # min angle at which rover calculates a turn
		self.angle_trim = 2.0  # global angle trim variable
		self.angle_trim_row = 2.0  # angle trim for straight driving
		self.angle_trim_curve = 15.0  # angle trim for curved driving

		self.look_ahead = 1.5  # this value navigated on test course well, but not after flag 
		self.look_ahead_row = 1.5
		self.look_ahead_curve = 0.5

		# Articulation settings:
		self.turn_left_val = 0  # publish this value to turn left
		self.turn_right_val = 2  # publish this value to turn right
		self.no_turn_val = 1  # publish this value to not turn??????

		# Actuator settings:
		self.actuator_min = -25  # accounting for scale factor on arduino (65 - 90) + 1 !!TEST THIS ONE!!
		self.actuator_max = 47  # accounting for scale factor on arduino (138 - 90) - 1
		self.actuator_home = 0
		self.actuator_stop = 0
		self.actuator_drive = None  # global actuator variable
		self.actuator_drive_slow = 20  # actuator setting for slow drive
		self.actuator_drive_med = 35  # actuator setting for medium drive

		# Throttle settings (updated 07/05/18):
		self.throttle_home = 120  # idle state
		self.throttle_min = 120  # lowest throttle state
		self.throttle_max = 60  # full throttle!
		self.throttle_drive = None  # global throttle variable
		self.throttle_drive_slow = 100  # throttle setting for slow driving
		self.throttle_drive_med = 80  # throttle setting for medium driving

		self.target_index = 0  # index in course that's the goal position
		
		self.current_goal = None  # [easting, northing] array
		self.current_pos = None  # [easting, northing] array
		self.current_angle = None  # angle from imu in radians

		self.np_course = None  # lazy np array version of course for certain manipulations

		self.at_flag = False  # todo: subscribe to at_flag topic?
		self.flag_index = None

		print("Red rover driver ready.")



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
		if flag_msg.data == True or flag_msg == True:
			print("Stopping cause we're at the flag!!!")
			self.at_flag = True  # sets main at_flag to True for robot..
		else:
			self.at_flag = False



	def start_driving_callback_multirow(self, msg):

		if msg.data == True:

			if not self.path_json:
				print("Waiting for drive node to be started..")
				return

			# starts following the first row in multirow course array:
			path_array = self.path_json['rows']

			self.target_index = 0

			print("Setting throttle and drive actuator to home states..")
			self.throttle_pub.publish(self.throttle_home)
			self.actuator_pub.publish(self.actuator_home)

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
		# print("Current angle: {}".format(self.current_angle))



	def run_basic_drive(self):
		"""
		Run a simple test for the big rover's linear actuation.
		"""
		print("Running actuator test for big rover..")
		rospy.sleep(1)

		print("Reving throttle up!")
		self.throttle_pub.publish(90)

		print("Pausing 5s before publishing to actuator..")
		rospy.sleep(5)

		print("Initiating drive.")
		# self.actuator_pub.publish(self.actuator_drive_slow)  # +1 from actuator home
		self.actuator_pub.publish(self.actuator_max)
		rospy.sleep(3)  # driving for as long as delay last

		print("Stopping rover by setting drive actuator to home state..")
		self.actuator_pub.publish(self.actuator_stop)  # set hydrolyic actuator to home state (aka stop)??
		print("Rover stopped, hopefully.")

		rospy.sleep(2)
		print("Calling mico leaf service to collect samples..")
		self.call_micoleaf_service(1)

		self.throttle_pub.publish(self.throttle_min)  # throttle back down

		return



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

		# if self.stop_gps:
		# 	self.wait_for_fix()


		# Iniates multirow loop, which loops the list of rows in path_array:
		for i in range(0, len(path_array) - 1):

			# Loops through row objects and starts following down first row in path array:

			row_array = path_array[i]['row']  # row array
			row_index = path_array[i]['index']  # row index

			# Flips row array if rover is facing opposite direction it was recorded:
			row_array = self.determine_drive_direction(row_array)

			self.np_course = np.array(row_array)

			_curr_utm = self.current_pos  # gets current utm
			init_target = self.calc_target_index(_curr_utm, 0, self.np_course[:,0], self.np_course[:,1])

			# Sets parameters for following a field row:
			self.angle_trim = self.angle_trim_row  # set angle trim to follow row (mostly straight)
			self.look_ahead = self.look_ahead_row
			self.throttle_drive = self.throttle_drive_med
			self.actuator_drive = self.actuator_drive_med
			
			# Starts following field row:
			self.execute_path_follow(row_array, init_target)  # follow down row

			# Calculates dubins curve from exit -> entry row (which is next row in course for this example):
			dubins_path = dp.handle_dubins(self.path_json, row_index, path_array[i+1]['index'])  # run dubins from current end or row to next row

			_curr_utm = self.current_pos  # gets current utm
			init_target = self.calc_target_index(_curr_utm, 0, dubins_path[:,0], dubins_path[:,1])

			# Sets parameters for following a dubins curve:
			self.angle_trim = self.angle_trim_curve  # set angle trim to follow curve
			self.look_ahead = self.look_ahead_curve
			self.throttle_drive = self.throttle_drive_slow
			self.actuator_drive = self.actuator_drive_slow
			
			# Starts following dubins curve:
			self.execute_path_follow(dubins_path.tolist(), init_target)  # like execute_path_follow, but with more sensitive parameters

			print("Finished dubins curve, following next row!")


		# Runs last row after above loop is finished:
		row_array = path_array[len(path_array) - 1]['row']  # row array
		row_index = path_array[len(path_array) - 1]['index']  # row index

		print("Following last row! Row {}".format(row_index))

		self.np_course = np.array(row_array)

		_curr_utm = self.current_pos  # gets current utm
		init_target = self.calc_target_index(_curr_utm, 0, self.np_course[:,0], self.np_course[:,1])

		# Sets parameters for following field row:
		self.angle_trim = self.angle_trim_row  # set angle trim to follow row (mostly straight)
		self.look_ahead = self.look_ahead_row
		self.throttle_drive = self.throttle_drive_med
		self.actuator_drive = self.actuator_drive_med

		# Starts following field row:
		self.execute_path_follow(row_array, init_target)  # follow down row



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





	def execute_path_follow(self, path_array, init_target):

		print("INITIAL TARGET: {}".format(init_target))

		self.np_course = np.array(path_array)  # sets numpy array of course\

		rospy.sleep(2)  # give messages time to publish

		_curr_utm = self.current_pos
		self.target_index = self.calc_target_index(_curr_utm, init_target, self.np_course[:,0], self.np_course[:,1])  # try using int_target
		self.current_goal = path_array[self.target_index]  # sets current goal

		# print("Total length of path array: {}".format(len(path_array)))
		# print("Initial target index: {}".format(self.target_index))
		# print("Initial target UTM: {}".format(self.current_goal))

		# Sleep routine for testing:
		print("Pausing 10 seconds before initiating driving (to have time to run out there)...")
		rospy.sleep(10)
		print("Starting driving routine.")



		##########################################################
		# TODO: CHANGE THESE TO BE GENERAL SO THEY CAN BE SET
		# BASED ON WHETHER IT'S STRAIGHT OR A CURVE.
		##########################################################



		print(">>> Reving up throttle before drive.")
		rospy.sleep(1)
		# self.throttle_pub.publish(self.throttle_drive_med)  # sets to 100
		self.throttle_pub.publish(self.throttle_drive)

		print(">>> Starting drive actuator to drive foward!")
		rospy.sleep(1)
		# self.actuator_pub.publish(self.actuator_drive_slow)  # sets to 20
		self.actuator_pub.publish(self.actuator_drive)


		###################################################################
		# This loop calculates a turn angle a look-ahead distance away,
		# then begins to execute the turn.
		###################################################################
		while not rospy.is_shutdown():

			if self.at_flag:
				print("At a flag in the course! Stopping the rover to take a sample.")
				self.execute_flag_routine()

			rospy.sleep(0.2)

			_curr_utm = self.current_pos  # gets current utm
			self.target_index = self.calc_target_index(_curr_utm, self.target_index, self.np_course[:,0], self.np_course[:,1])

			print("target index: {}".format(self.target_index))

			if self.target_index == None:
				print("Assuming end of course is reached! Stopping rover.")
				self.shutdown()
				return

			self.current_goal = self.np_course.tolist()[self.target_index]
			_curr_angle = self.current_angle  # gets current angle in radians

			A = (_curr_utm[0], _curr_utm[1], _curr_angle)
			B = (self.current_goal[0], self.current_goal[1], 0)  # note: B angle not used..

			turn_angle = -1.0*orientation_transforms.initiate_angle_transform(A, B)  # note: flipped sign of turn from imu

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



	def determine_drive_direction(self, current_row):
		"""
		Determines direction to drive down row.
		If rover is facing 180 degrees from direction row was
		recorded, then the course is flipped.
		"""
		angle_tolerance = math.radians(30)  # e.g., 180deg +/- 30deg

		rover_angle = math.radians(ot.transform_imu_frame(math.degrees(self.current_angle)))
		row_angle = dp.get_row_angle(current_row)  # gets angle of row

		angle_diff = abs(rover_angle - row_angle)  # calculates angle difference

		if angle_diff < math.pi + angle_tolerance and angle_diff > math.pi - angle_tolerance:
			# rover is facing opposite direction row was recorded, flips row array around:
			print("Flipping row array, rover is facing opposite direction from how it was recorded..")
			current_row_flipped = [pos for pos in reversed(current_row)]  # flipped array of easting, northing pairs
			return current_row_flipped
		
		return current_row
		


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



	def execute_flag_routine(self):
		"""
		Routine to run when the rover is at a flag.
		"""
		print("Making sure rover is stopped, then making request to take a sample..")
		rospy.sleep(0.1)
		self.actuator_pub.publish(self.actuator_stop)

		# Simulate sample collector service call with delay:
		########################################################################
		print("Pausing 10s to simulate a sample collection routine..")
		rospy.sleep(10)
		########################################################################


		# Call actual sample collector service here:
		########################################################################		
		# print("Pausing 5s, then calling mico leaf service..")
		# rospy.sleep(5)
		# self.throttle_pub.publish(self.throttle_max)
		# rospy.sleep(1)

		# print("Calling mico_leaf1 service.")
		# self.call_micoleaf_service()
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

		print(">>> Reving up throttle before drive.")
		rospy.sleep(1)
		self.throttle_pub.publish(self.throttle_drive_med)

		print(">>> Starting drive actuator to drive foward!")
		rospy.sleep(1)
		self.actuator_pub.publish(self.actuator_drive_slow)

		return



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
		last_angle = self.current_angle

		while abs(turn_angle) < abs(radians(goal_angle)) and not self.at_flag and not rospy.is_shutdown():


			self.articulator_pub.publish(_turn_val)

			rospy.sleep(1.0/self.rate)

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
		self.articulator_pub.publish(self.no_turn_val)
		rospy.sleep(1)
		self.throttle_pub.publish(self.throttle_home)
		rospy.sleep(1)
		print("Red rover stopped.")







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