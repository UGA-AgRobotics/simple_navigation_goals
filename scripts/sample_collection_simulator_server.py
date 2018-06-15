#!/usr/bin/env python

"""
"""

from simple_navigation_goals.srv import *
import rospy
from std_msgs.msg import Bool
import time



class SampleCollectorSimulator:

	def __init__(self):

		rospy.init_node('sample_collector_server')

		s = rospy.Service('start_sample_collection', SampleCollection, self.handle_sample_collection_request)

		self.collection_delay = 10  # simulated sample collection delay (units: seconds)

		print "Jackal sample collector node ready.."

		rospy.spin()


	def handle_sample_collection_request(self, msg):
		"""
		Starts sample collection routine when the sample_collector_server is
		requested
		"""
		# print("Incoming sample message: {}".format(msg))
		# print("Incoming message type: {}".format(type(msg)))

		if msg.sample_type == 'collect':
			# Collects data with leaf picker and soil sampler:
			print("Starting sample collection..")
			print("Delaying {} seconds..".format(self.collection_delay))
			time.sleep(self.collection_delay)
			print("Sample collection complete.")
			return True
		






if __name__ == '__main__':
	print("Starting sample_collector_server.")
	sample_collector = SampleCollectorSimulator()