#!/usr/bin/env python

import roslib
import rospy
from std_msgs.msg import Float64, Bool
from math import radians



class ArticulatorTestNode:

	def __init__(self):

		print("Starting articulator_test_node..")

		rospy.init_node('articulator_test_node', anonymous=True)

		self.current_pivot = None

		# Publishers:
		self.articulator_pub = rospy.Publisher('/driver/articulation_relay', Float64, queue_size=1)  # TODO: double check queue sizes..

		# Subscribers:
		rospy.Subscriber("/driver/encoder_velocity", Float64, self.rover_velocity_callback)
		rospy.Subscriber("/driver/pivot", Float64, self.rover_pivot_callback, queue_size=1)
		rospy.Subscriber("/driver/run_articulator_test", Bool, self.articulator_test_callback)

		# Articulation settings:
		self.turn_left_val = 0  # publish this value to turn left
		self.turn_right_val = 2  # publish this value to turn right

		self.no_turn = 1  # publish this value to not turn??????

		self.max_angle = 22  # max right (relative to driver/rover)
		self.min_angle = -22  # max left (relative to driver/rover)
		self.min_angle_tolerance = 1.0  # min allowable angle tolerance

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
			self.run_articulator_test_routine()



	def articulator_turn_callback(self, msg):
		"""
		Subscriber callback to initate a turn for testing
		the red rover's articulation.
		"""
		print("Received message on articulator_turn_callback to turn {} degrees..".format(msg.data))
		self.turn_to_angle(msg.data)




	def turn_left(self, goal_pivot):
		"""
		Turn left loop.
		"""
		while self.current_pivot > goal_pivot and not rospy.is_shutdown():
			print("Rover pivot: {}".format(msg))
			rospy.sleep(0.1)  # delay 100ms
			self.articulator_pub.publish(self.turn_left_val)  # turn left

		self.articulator_pub.publish(self.no_turn)

		return



	def turn_right(self, goal_pivot):
		"""
		Turn right loop.
		"""
		while self.current_pivot < goal_pivot and not rospy.is_shutdown():
			print("Rover pivot: {}".format(msg))
			rospy.sleep(0.1)  # delay 100ms
			self.articulator_pub.publish(self.turn_right_val)  # turn right

		self.articulator_pub.publish(self.no_turn)

		return




	def turn_to_angle(self, goal_pivot):
		"""
		Executes a turn to a specific angle using the
		/driver/pivot topic.
		Inputs:
			goal_pivot - angle to turn to based on rover's /driver/pivot topic
		"""

		# Trims goal angle based on min/max allowable angles:
		if goal_pivot > self.max_angle:
			print("Requested rover angle is too large, setting to max allowable: {}".format(self.max_angle))
			goal_pivot = self.max_angle
		elif goal_pivot < self.min_angle:
			print("Request rover angle is too small, setting to min allowable: {}".format(self.min_angle))
			goal_pivot = self.min_angle

		turn_angle = goal_pivot - self.current_pivot  # determines direction to turn
		print("Turning {} degrees..".format(turn_angle))

		rospy.spin(1)

		if turn_angle < -self.min_angle_tolerance:
			self.turn_left(goal_pivot)  # start turning left
		elif turn_angle > self.min_angle_tolerance:
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
		self.turn_to_angle(0.0)

		print("Turning the rover 10 degrees left..")
		rospy.sleep(5)
		self.turn_to_angle(-10.0)

		print("Re-centering the rover..")
		rospy.sleep(5)
		self.turn_to_angle(0.0)

		print("Turning the rover 10 degrees right..")
		rospy.sleep(5)
		self.turn_to_angle(10.0)		

		print("Re-centering the rover..")
		rospy.sleep(5)
		self.turn_to_angle(0.0)

		print("Articulation test complete.")







if __name__ == '__main__':

	try:
		ArticulatorTestNode()
	except rospy.ROSInterruptException:
		raise