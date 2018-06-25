#!/usr/bin/env python

import roslib
import rospy
# from geometry_msgs.msg import Twist, Point, Quaternion
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
# from std_msgs.msg import String
# from std_msgs.msg import Float64
# from simple_navigation_goals.srv import *



# Global settings:
linear_speed = 0.2  # units of m/s
rate = 20  # Hz



class DriveNode:

	def __init__(self):

		print("Starting drive_node..")

		rospy.init_node('drive_node', anonymous=True)

		# Publishers:
		self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

		# Subscribers:
		rospy.Subscriber("/rf_stop", Bool, self.rf_stop_callback, queue_size=1)
		rospy.Subscriber("/start_driving", Bool, self.start_driving_callback, queue_size=1)  # sub to /at_flag topic from jackal_flags_node.py
		rospy.Subscriber('/at_flag', Bool, self.flag_callback, queue_size=1)

		self.move_cmd = Twist()

		self.emergency_stop = False
		self.at_flag = False

		print("drive_node ready.")

		rospy.spin()



	def start_driving_callback(self, start_msg):
		"""
		Initiates continuous driving routine. Starts when True is received
		from /start_driving topic.
		"""
		if start_msg.data == True:
			print("Starting driving routine!")
			self.continuous_drive()
		else:
			print("Stopping driving routine!")
			self.cmd_vel.publish(Twist())



	def flag_callback(self, flag_msg):
		"""
		Subscribes to /at_flag topic that's being published by
		jackal_flag_node.py. Needs to stop Jackal if at_flag is True
		"""
		# print("At flag? {}".format(flag_msg.data))

		if flag_msg.data == True:
			print("Stopping cause we're at the flag!!!")
			self.at_flag = True  # sets main at_flag to True for robot..



	def rf_stop_callback(self, stop_msg):
		"""
		/rf_stop is an emergency stop from the arduino, which uses a 
		32197-MI 4 Ch. remote control receiver
		"""
		# print("Received RF stop message! {}".format(stop_msg))
		if stop_msg.data == True:
			print("Receiving RF stop!")
			self.emergency_stop = True
		else:
			self.emergency_stop = False



	def continuous_drive(self):

		self.move_cmd.linear.x = linear_speed

		while not rospy.is_shutdown():

			while not self.emergency_stop:

				print("Publishing to /cmd_vel..")

				self.cmd_vel.publish(self.move_cmd)
				rospy.sleep(1.0/rate)

			print("Emergency stop received! Publishing blank Twist() to stop robot..")
			self.cmd_vel.publish(Twist())

		print("ROS shutting down.. drive_node publishing blank Twist() to stop robot..")
		self.cmd_vel.publish(Twist())
		print("Robot stopped.")







if __name__ == '__main__':

	try:
		DriveNode()
	except rospy.ROSInterruptException:
		raise