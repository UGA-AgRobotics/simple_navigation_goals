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
		self.throttle_low = 120  # lowest rpm
		self.throttle_high = 60  # highest rpm

		# Publishers:
		self.throttle_pub = rospy.Publisher('/driver/throttle', UInt8, queue_size=1)  # TODO: double check queue sizes..

		# Subscribers:
		rospy.Subscriber("/driver/encoder_velocity", Float64, self.rover_velocity_callback)
		rospy.Subscriber("/driver/pivot", Float64, self.rover_pivot_callback, queue_size=1)
		rospy.Subscriber("/driver/test/set_throttle", Float64, self.set_throttle_callback, queue_size=1)

		print("Setting throttle to min value to idle..")

		self.throttle_pub.publish(self.throttle_home)  # set throttle to lowest rpm at startup

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



	def set_throttle_callback(self, msg):
		"""
		Subscriber callback for running throttle test.
		"""
		
		throttle_val = msg.data

		print("Setting throttle to {}".format(throttle_val))
		print("Publishing single value, {}, to /driver/throttle topic".format(self.throttle_low))

		if throttle_val < self.throttle_high or throttle_val > self.throttle_low:
			print("!!! Must provide a throttle value between {} (full throttle) and {} (low throttle) !!!".format(self.throttle_high, self.throttle_low))
			return

		if not throttle_val:
			throttle_val = self.throttle_home

		self.throttle_pub.publish(throttle_val)



	# def run_throttle_test_routine(self, throttle_val=None):
	# 	"""
	# 	Runs a throttle test routine for the big rover.
	# 	"""
	# 	print("Running thottle test for big rover..")
	# 	print("Publishing single value, {}, to /driver/throttle topic".format(self.throttle_low))

	# 	if throttle_val < self.throttle_high or throttle_val > self.throttle_low:
	# 		print("!!! Must provide a throttle value between {} (full throttle) and {} (low throttle) !!!".format(self.throttle_high, self.throttle_low))
	# 		throttle_val = None

	# 	if not throttle_val:
	# 		print("Setting throttle to idle (home) state..")
	# 		throttle_val = self.throttle_home

	# 	self.throttle_pub.publish(self.throttle_val)



	def shutdown_throttle(self):
		"""
		Set throttle to home state if node is killed/shutdown.
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