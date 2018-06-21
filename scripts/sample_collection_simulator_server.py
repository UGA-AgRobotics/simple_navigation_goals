#!/usr/bin/env python

"""
"""

from simple_navigation_goals.srv import *
import rospy
from std_msgs.msg import Bool
import time
# from arduino_controller import ArduinoController



class SampleCollectorSimulator:

	def __init__(self):

		rospy.init_node('sample_collector_server')

		s = rospy.Service('start_sample_collection', SampleCollection, self.handle_sample_collection_request)

		self.collection_delay = 10  # simulated sample collection delay (units: seconds)

		# self.arduino_serial_port = rospy.get_param('ARDUINO_SERIAL_PORT', '/dev/ttyACM2')  # tty port for Arduino
		# self.arduino_baud = rospy.get_param('ARDUINO_BAUD', 9600)  # baud rate for arduino serial communication
		# self.arduino_controller = ArduinoController(self.arduino_serial_port, self.arduino_baud)


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
			# self.arduino_controller.arduino.write('flag')
			time.sleep(self.collection_delay)
			# self.arduino_controller.arduino.write('flagoff')
			print("Sample collection complete.")
			return True
		






if __name__ == '__main__':
	print("Starting sample_collector_server..")
	sample_collector = SampleCollectorSimulator()