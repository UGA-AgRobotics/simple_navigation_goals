#!/usr/bin/env python

import roslib
import rospy
from std_msgs.msg import Float64, Bool
from math import radians
from simple_navigation_goals.srv import *



class ArticulatorTestNode:

	def __init__(self):

		print("Starting articulator_test_node..")

		rospy.init_node('articulator_test_node', anonymous=True)

		rospy.on_shutdown(self.shutdown_articulation)

		self.current_pivot = None

		# Publishers:
		self.articulator_pub = rospy.Publisher('/driver/articulation_relay', Float64, queue_size=1)  # TODO: double check queue sizes..

		# Subscribers:
		rospy.Subscriber("/driver/encoder_velocity", Float64, self.rover_velocity_callback)
		rospy.Subscriber("/driver/pivot", Float64, self.rover_pivot_callback, queue_size=1)
		rospy.Subscriber("/driver/test/run_articulator_test", Bool, self.articulator_test_callback)
		rospy.Subscriber("/driver/test/execute_turn", Float64, self.articulator_turn_callback)

		# Services:
		# print("Waiting for get_red_rover_rot service..")
		# rospy.wait_for_service('get_red_rover_rot')
		# self.get_red_rover_rot = rospy.ServiceProxy('get_red_rover_rot', RedRoverRot)
		# print("get_red_rover_rot service ready.")
		print("Waiting for get_jackal_rot service..")
		rospy.wait_for_service('get_jackal_rot')
		self.get_jackal_rot = rospy.ServiceProxy('get_jackal_rot', JackalRot)
		print("get_jackal_rot service ready.")

		# Articulation settings:
		self.turn_left_val = 0  # publish this value to turn left
		self.turn_right_val = 2  # publish this value to turn right
		self.no_turn_val = 1  # publish this value to not turn??????

		self.max_pivot = 22  # max right (relative to driver/rover)
		self.min_pivot = -22  # max left (relative to driver/rover)
		self.min_pivot_tolerance = 1.0  # min allowable angle tolerance

		print("articulator_test_node ready.")

		rospy.spin()



	def rover_velocity_callback(self, msg):
		"""
		Subscriber callback for big rover's velocity.
		"""
		# print("Rover velocity callback message: {}".format(msg))
		pass
		


	def rover_pivot_callback(self, msg):
		"""
		Subscriber callback for the big rover's current angle/pivot.
		"""
		# print("Rover pivot callback message: {}".format(msg))
		self.current_pivot = msg.data;



	def articulator_test_callback(self, msg):
		"""
		Subscriber callback for running articulation test.
		"""
		if msg.data == True:
			# self.run_articulator_test_routine()
			self.run_imu_turn_test()



	def articulator_turn_callback(self, msg):
		"""
		Subscriber callback to initate a turn for testing
		the red rover's articulation.
		"""
		print("Received message on articulator_turn_callback to turn {} degrees..".format(msg.data))
		self.turn_to_pivot(msg.data)



	def check_pivot_bounds(self, goal_pivot):
		"""
		Trims goal pivot based on min/max allowable angles:
		"""
		if goal_pivot > self.max_pivot:
			print("Requested rover pivot is too large, setting to max allowable: {}".format(self.max_pivot))
			goal_pivot = self.max_pivot
		elif goal_pivot < self.min_pivot:
			print("Request rover pivot is too small, setting to min allowable: {}".format(self.min_pivot))
			goal_pivot = self.min_pivot
		return goal_pivot



	def turn_left(self, goal_pivot):
		"""
		Turn left loop.
		"""
		while self.current_pivot > goal_pivot and not rospy.is_shutdown():
			print("Rover pivot: {}".format(self.current_pivot))
			rospy.sleep(0.1)  # delay 100ms
			self.articulator_pub.publish(self.turn_left_val)  # turn left

		self.articulator_pub.publish(self.no_turn_val)

		return



	def turn_right(self, goal_pivot):
		"""
		Turn right loop.
		"""
		while self.current_pivot < goal_pivot and not rospy.is_shutdown():
			print("Rover pivot: {}".format(self.current_pivot))
			rospy.sleep(0.1)  # delay 100ms
			self.articulator_pub.publish(self.turn_right_val)  # turn right

		self.articulator_pub.publish(self.no_turn_val)

		return




	def turn_to_pivot(self, goal_pivot):
		"""
		Executes a turn to a specific angle using the
		/driver/pivot topic.
		Inputs:
			goal_pivot - angle to turn to based on rover's /driver/pivot topic
		"""

		goal_pivot = self.check_pivot_bounds(goal_pivot)

		turn_angle = goal_pivot - self.current_pivot  # determines direction to turn
		print("Turning {} degrees..".format(turn_angle))

		rospy.sleep(1)

		if turn_angle < -self.min_pivot_tolerance:
			self.turn_left(goal_pivot)  # start turning left
		elif turn_angle > self.min_pivot_tolerance:
			self.turn_right(goal_pivot)  # start turning right
		else:
			print("Turn angle is zero, canceling turn request..")
			return  # don't turn if angle is 0



	def run_articulator_test_routine(self):
		"""
		Run a simple test for the red rover's linear actuation.
		"""
		print("Running articulation test for red rover..")
	
		print("Centering the rover first..")
		rospy.sleep(5)
		self.turn_to_pivot(0.0)

		print("Turning the rover 10 degrees left..")
		rospy.sleep(5)
		self.turn_to_pivot(-10.0)

		print("Re-centering the rover..")
		rospy.sleep(5)
		self.turn_to_pivot(0.0)

		print("Turning the rover 10 degrees right..")
		rospy.sleep(5)
		self.turn_to_pivot(10.0)		

		print("Re-centering the rover..")
		rospy.sleep(5)
		self.turn_to_pivot(0.0)

		print("Articulation test complete.")



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
		last_angle = self.get_jackal_rot().jackal_rot  # get angle from IMU (in radians)

		while abs(turn_angle) < abs(goal_angle) and not self.at_flag and not rospy.is_shutdown():

			# self.cmd_vel.publish(move_cmd)

			# print("Current angle: {}, Current pivot: {}".format(self.last_angle, self.current_pivot))

			self.articulator_pub.publish(_turn_val)

			rospy.sleep(1.0/rate)

			curr_angle = self.get_jackal_rot().jackal_rot
			delta_angle = self.normalize_angle(curr_angle - last_angle)
			turn_angle += delta_angle
			last_angle = curr_angle

			if delta_angle == 0.0:
				print("Delta angle is 0, breaking out of turning loop..")
				break

		# if self.emergency_stop:
		# 	print("Emergency stop from RF remote received, stopping turning routine..")

		return



	# def imu_turn_controller(self):
	# 	"""
	# 	An abbreviated version of the p2p_drive_routine in basic_drive_6.
	# 	Determines turn angle with IMU between it's position, A, and the
	# 	goal position, B.
	# 	"""

	# 	curr_pose_utm = self.nav_controller.get_current_position()
	# 	curr_angle = self.nav_controller.get_jackal_rot().jackal_rot

	# 	print("Jackal's position in UTM: {}, Jackal's angle: rad-{}, deg-{}".format(curr_pose_utm, curr_angle, degrees(curr_angle)))

	# 	print("GOAL POSITION: {}".format(goal_pos))

	# 	A = (curr_pose_utm[0], curr_pose_utm[1], curr_angle)
	# 	B = (goal_pos[0], goal_pos[1], goal_pos[2])  # NOTE: B's orientation currently hardcoded for testing..

	# 	# NOTE: perhaps add angle tolerance here, e.g., if turn_angle is
	# 	# 0 +/- angle_tolerance, then don't turn the rover!
	# 	##########################################################################
	# 	turn_angle = orientation_transforms.initiate_angle_transform(A, B)

	# 	print("Turn angle, pre-tolerance filter: {}".format(turn_angle))

	# 	if abs(turn_angle) > abs(self.angle_tolerance):
	# 	# if turn_angle != 0:
	# 		# Determine angle to turn based on IMU..
	# 		print("Telling Jackal to turn {} degreess..".format(turn_angle))
	# 		self.nav_controller.execute_turn(radians(turn_angle))
	# 		print("Finished turn.")



	def run_imu_turn_test(self):
		"""
		For this test, I'd like the rover to center itself, then run the following sequence:
		1. Read in pivot value and straighten the rover.
		2. Read in IMU angle.
		3. Turn left a magnitude of 5 degrees using the IMU angle service.
		4. Turn 5 degrees right to theoretically straighten the rover.
		5. Turn 5 more degrees right using the IMU angle service.
		6. Turn 5 degrees left to theoretically straighten the rover again.
		7. Print out the pivot value to compare the initial and final pivot value (should be close).
		"""
		print("Running articulation test for red rover..")

		rospy.sleep(1)
		print("Current pivot: {}".format(self.current_pivot))
	
		print("Centering the rover first..")
		rospy.sleep(5)
		self.turn_to_pivot(10.0)  # NOTE: IF THE SENSOR IS STILL OFF, SHIFT THIS VALUE TO ACCOUNT FOR THAT!!!!!

		rospy.sleep(5)
		print("Turning rover 5 degrees left using the IMU..")
		rospy.sleep(2)
		self.translate_angle_with_imu(-5.0)
		print("Turn complete.")

		rospy.sleep(5)
		print("Now turning rover back to center using the IMU..")
		rospy.sleep(2)
		self.translate_angle_with_imu(5.0)
		print("Turn complete.")

		rospy.sleep(5)
		print("Now turning rover 5 degrees right using the IMU..")
		rospy.sleep(2)
		self.translate_angle_with_imu(5.0)
		print("Turn complete.")

		rospy.sleep(5)
		print("Now turning rover back to center using the IMU..")
		rospy.sleep(2)
		self.translate_angle_with_imu(-5.0)
		print("Turn complete.")

		rospy.sleep(2)
		print("IMU turn test complete.")
		print("Final pivot: {}".format(self.current_pivot))

		return



	def normalize_angle(self, angle):
		res = angle
		while res > pi:
			res -= 2.0 * pi
		while res < -pi:
			res += 2.0 * pi
		return res



	def shutdown_articulation(self):
		"""
		Stops turning if node is killed/shutdown.
		"""
		print(">>> Stopping red rover articulation..")
		rospy.sleep(1)
		self.articulator_pub.publish(self.no_turn_val)
		rospy.sleep(1)
		print(">>> Red rover articulation stopped.")







if __name__ == '__main__':

	try:
		ArticulatorTestNode()
	except rospy.ROSInterruptException:
		raise