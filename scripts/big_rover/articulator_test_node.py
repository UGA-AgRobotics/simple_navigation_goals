#!/usr/bin/env python

import roslib
import rospy
from std_msgs.msg import Float64



class ArticulatorTestNode:

	def __init__(self):

		print("Starting articulator_test_node..")

		rospy.init_node('articulator_test_node', anonymous=True)

		# Publishers:
		# self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.articulator_pub = rospy.Publisher('/driver/articulation_relay', Float64, queue_size=1)  # TODO: double check queue sizes..

		# Subscribers:
		rospy.Subscriber("/driver/encoder_velocity", Float64, self.rover_velocity_callback)
		rospy.Subscriber("/driver/pivot", Float64, self.rover_pivot_callback, queue_size=1)

		print("articulator_test_node ready.")

		rospy.spin()



	def rover_velocity_callback(self, msg):
		"""
		Subscriber callback for big rover's velocity.
		"""
		print("Rover velocity callback message: {}".format(msg))
		


	def rover_pivot_callback(self, msg):
		"""
		Subscriber callback for the big rover's current angle/pivot.
		"""
		print("Rover pivot callback message: {}".format(msg))



	def run_articulator_test_routine(self):
		"""
		Run a simple test for the big rover's linear actuation.
		"""
		print("Running articulator test for big rover..")
		pass







if __name__ == '__main__':

	try:
		ArticulatorTestNode()
	except rospy.ROSInterruptException:
		raise