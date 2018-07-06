#!/usr/bin/env python

import roslib
import rospy
from std_msgs.msg import Float64, UInt8, Bool



class ThrottleTestNode:

	def __init__(self):

		print("Starting throttle_test_node..")

		rospy.init_node('throttle_test_node', anonymous=True)

		rospy.on_shutdown(self.shutdown_throttle)

		self.throttle_home = 120
		self.throttle_min = 120
		self.throttle_max = 60

		# Publishers:
		self.throttle_pub = rospy.Publisher('/driver/throttle', UInt8, queue_size=1)  # TODO: double check queue sizes..

		# Subscribers:
		rospy.Subscriber("/driver/encoder_velocity", Float64, self.rover_velocity_callback)
		rospy.Subscriber("/driver/pivot", Float64, self.rover_pivot_callback, queue_size=1)
		rospy.Subscriber("/driver/test/run_throttle_test", Bool, self.throttle_test_callback)

		print("Setting throttle to min value to idle..")

		self.throttle_pub.publish(self.throttle_home)  # set throttle to lowest rpm to idle

		print("throttle_test_node ready.")

		# self.run_throttle_test_routine()

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
		pass



	def throttle_test_callback(self, msg):
		"""
		Subscriber callback for running throttle test.
		"""
		if msg.data == True:
			self.run_throttle_test()



	def run_throttle_test_routine(self, throttle_val):
		"""
		Runs a throttle test routine for the big rover.
		"""
		print("Running thottle test for big rover..")
		print("Publishing single value, {}, to /driver/throttle topic".format(self.throttle_min))

		if throttle_val < self.throttle_max or throttle_val > self.throttle_min:
			print("!!! Must provide a throttle value between {} (full throttle) and {} (low throttle) !!!".format(self.throttle_max, self.throttle_min))
			return

		self.throttle_pub.publish(self.throttle_val)

		# while not rospy.is_shutdown():
		# 	rospy.sleep(0.2)  # sleep for 200 ms
		# 	self.throttle_pub.publish(self.throttle_min)
		# 	print("Publishing {} to /driver/throttle topic..".format(self.throttle_min))



	def shutdown_throttle(self):
		"""
		Set throttle to home state if node is killed.
		"""
		print("Shutting down throttle, setting it to home state: {}".format(self.throttle_home))
		rospy.sleep(1.0)
		self.throttle_pub.publish(self.throttle_home)
		rospy.sleep(1.0)
		print("Throttle set to home state.")







if __name__ == '__main__':

	try:
		ThrottleTestNode()
	except rospy.ROSInterruptException:
		raise