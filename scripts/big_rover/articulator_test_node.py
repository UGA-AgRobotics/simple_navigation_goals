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
		self.current_pivot = msg.data;



	def articulator_test_callback(self, msg):
		"""
		Subscriber callback for running articulation test.
		"""
		if msg.data == True:
			self.run_articulator_test_routine()



	def run_articulator_test_routine(self):
		"""
		Run a simple test for the big rover's linear actuation.
		"""
		print("Running articulator test for big rover..")
		
		rospy.sleep(1)

		print("Turning left 15 degrees..")
		while self.current_pivot < radians(15) and not rospy.is_shutdown():
			print("Current angle: {}".format(self.current_pivot))
			rospy.sleep(0.1)
			self.articulator_pub.publish(0)  # turn left

		rospy.sleep(1)

		print("Now turning right 15 degrees..")
		while self.current_pivot < radians(15) and not rospy.is_shutdown():
			print("Current angle: {}".format(self.current_pivot))
			rospy.sleep(0.1)
			self.articulator_pub.publish(2)  # turn right







if __name__ == '__main__':

	try:
		ArticulatorTestNode()
	except rospy.ROSInterruptException:
		raise