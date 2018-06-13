#!/usr/bin/env python

"""
Acts as the sample collection node for testing the full navigation routine.
The robot will drive a course to a flag, then publish a /collect_sample boolean
to this node, which will simply pause for some amount of time, and publish a true
boolean to /sample_collected once said delay is over.
"""

import rospy
from std_msgs.msg import Bool
import time



class SampleCollectorSimulator:

	def __init__(self):

		self.collection_delay = 10  # simulated sample collection delay (units: seconds)

		self.sample_publisher = rospy.Publisher('/sample_collected', Bool, queue_size=1)  # publishes True when sample is collected
		self.sample_subscriber = rospy.Subscriber('/collect_sample', Bool, self.sample_callback queue_size=1)  # subs to collect_sample to know when to start sample collection

		

	def sample_callback(self, msg):
		"""
		Subscribes to /collect_sample topic of Bool type.
		When True, initiates sample collection routine, which for this
		module is just a delay
		"""
		print("Incoming sample message: {}".format(msg))

		if msg.data == True:
			print("Received True, starting sample collection..")
			print("Delaying {} seconds..".format(self.collection_delay))
			time.sleep(self.collection_delay)
			print("Sample collection complete.")
			print("Sending True on /sample_collected back to robot to continue on to the next flag..")
			self.sample_publisher.publish(True)