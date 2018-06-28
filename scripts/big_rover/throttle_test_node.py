#!/usr/bin/env python

import roslib
import rospy
from std_msgs.msg import Float64, UInt8



class ThrottleTestNode:

	def __init__(self):

		print("Starting throttle_test_node..")

		rospy.init_node('throttle_test_node', anonymous=True)

		# Publishers:
		# self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.throttle_pub = rospy.Publisher('/driver/throttle', UInt8, queue_size=1)  # TODO: double check queue sizes..

		# Subscribers:
		rospy.Subscriber("/driver/encoder_velocity", Float64, self.rover_velocity_callback)
		rospy.Subscriber("/driver/pivot", Float64, self.rover_pivot_callback, queue_size=1)

		print("throttle_test_node ready.")

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



	def run_throttle_test_routine(self):
		"""
		Runs a throttle test routine for the big rover.
		"""
		print("Running thottle test for big rover..")
		pass







if __name__ == '__main__':

	try:
		ThrottleTestNode()
	except rospy.ROSInterruptException:
		raise