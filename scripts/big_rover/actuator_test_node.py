#!/usr/bin/env python

import roslib
import rospy
from std_msgs.msg import Float64, Bool



class ActuatorTestNode:

	def __init__(self):

		print("Starting actuator_test_node..")

		rospy.init_node('actuator_test_node', anonymous=True)

		rospy.on_shutdown(self.shutdown_actuator)

		self.actuator_min = 65
		self.actuator_max = 138
		self.actuator_scale = 90
		self.actuator_home = 90

		self.actuator_test_val = 30  # note: arduino firmware code will write 91 to servo

		# Services:
		# s = rospy.Service('test_drive', DriveDistance, self.handle_test_drive)

		# Publishers:
		self.actuator_pub = rospy.Publisher('/driver/linear_drive_actuator', Float64, queue_size=1)  # TODO: double check queue sizes..

		# Subscribers:
		rospy.Subscriber("/driver/encoder_velocity", Float64, self.rover_velocity_callback)
		rospy.Subscriber("/driver/pivot", Float64, self.rover_pivot_callback, queue_size=1)
		rospy.Subscriber("/driver/run_actuator_test", Bool, self.actuator_test_callback)

		print("actuator_test_node ready.")

		# self.run_actuator_test_routine()

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



	def actuator_test_callback(self, msg):
		"""
		Subscriber for starting drive test.
		"""
		if msg.data == True:
			self.run_actuator_test_routine()



	# def handle_test_drive(self, req):
	# 	"""
	# 	Function for test_drive service. Drives for a number
	# 	of seconds.
	# 	Input - drive_time (type: float64).
	# 	"""
	# 	drive_time = req.drive_time  # this currently has a default value of 1 in .srv file
	# 	actuator_val = req.actuator_val  # this currently has a default value of 1 in .srv file
		
	# 	print("Initiating test drive routine. Driving {} seconds then stopping..".format(drive_time))

	# 	return
		


	def run_actuator_test_routine(self):
		"""
		Run a simple test for the big rover's linear actuation.
		"""
		print("Running actuator test for big rover..")

		print("Pausing 5s before publishing to actuator..")
		rospy.sleep(5)

		print("Publishing ")
		self.actuator_pub.publish(self.actuator_test_val)  # +1 from actuator home

		rospy.sleep(2)  # sleep for 1s

		print("Stopping rover by setting drive actuator to home state..")

		self.actuator_pub.publish(self.actuator_home)  # set hydrolyic actuator to home state (aka stop)??

		print("Rover stopped, hopefully.")

		return



	def shutdown_actuator(self):
		"""
		Set actuator to home state if node is killed/shutdown.
		"""
		print("Shutting down red rover actuator..")
		rospy.sleep(1)
		self.actuator_pub.publish(self.actuator_home)
		rospy.sleep(1)
		print("Red rover actuator shutdown.")







if __name__ == '__main__':

	try:
		ActuatorTestNode()
	except rospy.ROSInterruptException:
		raise