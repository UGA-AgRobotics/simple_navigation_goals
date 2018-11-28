#!/usr/bin/env python

from socketIO_client import SocketIO, LoggingNamespace
import logging
import serial  # NOTE: This is pySerial library and not serial
import time
import sys
import roslib
import rospy
import json
from std_msgs.msg import String, Bool
from sensor_msgs.msg import NavSatFix



class EmlidSocketIOClient:

	def __init__(self, emlid_ip=None, emlid_port=None, test_routine=False):

		print("Starting emlid_socketio_client node..")

		rospy.init_node('emlid_socketio_client', anonymous=True, disable_signals=True)

		# Subscribers:


		# Publishers:
		self.solution_status_publisher = rospy.Publisher("/emlid_solution_status", String, queue_size=1)
		self.gps_stop_publisher = rospy.Publisher("/stop_gps", Bool, queue_size=1)  # stop until GPS gets Fix again
		# self.gps_pos_publisher = rospy.Publisher('/gps_pos',
		self.gps_pos_publisher = rospy.Publiser("/fix", NavSatFix, queue_size=1)

		self.emlid_ip = emlid_ip or '192.168.131.201'
		self.emlid_port = emlid_port or 80

		print("Reach IP: {}, Reach Port: {}".format(self.emlid_ip, self.emlid_port))

		self.emlid_keys = [
			'solution status',
			'baseline length float (m)',
			'ratio for ar validation',
			'age of differential (s)',
			'pos llh (deg,m) base',
			'pos llh single (deg,m) rover',
			'# of satellites base',
			'# of satellites rover'
		]
		self.rover_pos_key = 'pos llh single (deg,m) rover'
		self.rover_status_key = 'solution status'
		self.rover_ar_ratio = 'ratio for ar validation'

		self.status_options = ['fix', 'float', 'single', '-']
		self.off_signal = 'off'  # turn all LEDs off on arduino
		self.flag_on_signal = 'flag'
		self.flag_off_signal = 'flagoff'

		# Runs socketio server if not running a test routine:
		# if test_routine:
		# 	self.run_light_test()
		# else:
		# 	self.connect_to_socketio_server()  # initiate connection to emlid's socketio server
		# self.connect_to_socketio_server()

		rospy.sleep(2)

		print("Connecting to SocketIO server from Emlid reach unit..")

		with SocketIO(self.emlid_ip, self.emlid_port, LoggingNamespace) as socketIO:

			socketIO.on('connect', self.on_connect)
			socketIO.on('disconnect', self.on_disconnect)
			socketIO.on('reconnect', self.on_reconnect)
			socketIO.on('status broadcast', self.on_status_broadcast)

			rospy.sleep(1)

			print("emlid_socketio_client node ready..")

			# rospy.spin()
			socketIO.wait()



	def on_connect(self):
		print('connected.')



	def on_disconnect(self):
		print("Emlid node's socketio connection disconnected.. Sending message to stop rover on /stop_gps..")
		self.gps_stop_publisher.publish(True)

		print("Shutting down socketio client ROS node..")
		rospy.signal_shutdown("Socketio server disconnected, shutting down emlid_socketio_client ROS node.")
		
		print('disconnected.')



	def on_reconnect(self):
		print('reconnected.')



	def on_status_broadcast(self, msg):

		try:
			emlid_data = json.loads(json.dumps(msg))  # re-serialize for the safety (ensures it's JSON-safe data coming in)
		except Exception as e:
			logging.warning("Could not reserialize incoming JSON..")
			raise

		if not msg:
			print("No message from Emlid socket.io server.. Sending stop message to rover on /stop_gps..")
			self.gps_stop_publisher.publish(True)
			return

		print("GPS Position: {}".format(emlid_data[self.rover_pos_key]))
		print("GPS Status: {}".format(emlid_data[self.rover_status_key]))
		print("GPS AR Validation Ratio: {}".format(emlid_data[self.rover_ar_ratio]))

		self.publish_emlid_pos(emlid_data)



	def publish_emlid_pos(self, emlid_data):
		"""
		Parses the lat,lon,alt string into a NavSatFix object, 
		and publishes to /fix topic.
		"""

		emlid_list = emlid_data.split(',')
		emlid_lat = emlid_list[0]
		emlid_lon = emlid_list[1]
		emlid_alt = emlid_list[2]

		print("Emlid lat,lon,alt items: {}, {}, {}".format(emlid_lat, emlid_lon, emlid_alt))

		navsat_pos = NavSatFix()
		navsat_pos['latitude'] = emlid_lat
		navsat_pos['longitude'] = emlid_lon
		navsat_pos['altitude'] = emlid_alt

		print("Publishing to /fix: {}".format(navsat_pos))

		self.gps_pos_publisher.publish(navsat_pos)



	def check_for_fix(self, status):
		"""
		Sends True on /stop_fix topic if Emlid has anything
		other than a fix.
		"""
		if not status in self.status_options:
			print("Status not recognized.. Stopping rover..")
			self.gps_stop_publisher.publish(True)
			return False

		if status == 'fix':
			self.gps_stop_publisher.publish(False)  # starts rover after getting fix
			return True

		else:
			print(">>> Emlid node sending message on /stop_gps to stop rover due to losing GPS fix..")
			self.gps_stop_publisher.publish(True)  # stops rover to get fix
			return False



if __name__ == '__main__':

	_emlid_ip = sys.argv[1]
	_emlid_port = sys.argv[2]

	try:
		# Starts Emlid Reach RS SocketIO Client:
		emlidsock = EmlidSocketIOClient(_emlid_ip, _emlid_port)
	except rospy.ROSInterruptException:
		raise
