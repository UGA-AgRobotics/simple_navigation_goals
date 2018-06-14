#!/usr/bin/env python

"""
Acts as the sample collection node for testing the full navigation routine.
The robot will drive a course to a flag, then publish a /collect_sample boolean
to this node, which will simply pause for some amount of time, and publish a true
boolean to /sample_collected once said delay is over.
"""

from simple_navigation_goals.srv import *
import rospy
from std_msgs.msg import Bool
import time



class SampleCollectorSimulator:

	def __init__(self):


		rospy.init_node('sample_collector_server')
		s = rospy.Service('start_sample_collection', SampleCollection, handle_sample_collection_request)

		self.collection_delay = 10  # simulated sample collection delay (units: seconds)
		# self.sample_publisher = rospy.Publisher('/sample_collected', Bool, queue_size=1)  # publishes True when sample is collected
		# self.sample_subscriber = rospy.Subscriber('/collect_sample', Bool, self.sample_callback, queue_size=1)  # subs to collect_sample to know when to start sample collection

		print "sample_ subscribed to /fix from Jackal.."
		print "Jackal sample collector node ready.."

		rospy.spin()


	def handle_sample_collection_request(self, msg):
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
			# self.sample_publisher.publish(True)
			return True
		






if __name__ == '__main__':
	print("Starting sample_collector_server.")
	sample_collector = SampleCollectorSimulator()









# global_current_position = None
# global_current_orientation = None



# def pos_callback(data):
# 	"""
# 	Position callback, which is executed in the event that a GPS fix is
# 	published by the Jackal.
# 	"""
# 	global global_current_position
# 	global_current_position = data
# 	print "jackal_pos_server: jackal's position: {}".format(global_current_position)


# def handle_pos_request(req):
# 	print "Incoming drive request: {}".format(req)
# 	global global_current_position
# 	return JackalPosResponse(global_current_position)  # Return current position of Jackal..



# def get_jackal_pos_server():
# 	rospy.init_node('get_jackal_pos_server')
# 	s = rospy.Service('get_jackal_pos', JackalPos, handle_pos_request)
# 	rospy.Subscriber('/fix', NavSatFix, pos_callback, queue_size=1)  # Subscribe to Jackal's /fix topic (reach units)
# 	print "get_jackal_pos_server subscribed to /fix from Jackal.."
# 	print "Jackal pos server ready.."
# 	rospy.spin()


# if __name__ == "__main__":
# 	get_jackal_pos_server()