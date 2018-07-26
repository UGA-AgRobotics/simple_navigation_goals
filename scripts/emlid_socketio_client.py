#!/usr/bin/env python

from socketIO_client import SocketIO, LoggingNamespace
import logging
import serial  # NOTE: This is pySerial library and not serial
import time
import sys
import roslib
import rospy
from std_msgs.msg import String



class EmlidSocketIOClient:

	def __init__(self, reach_ip=None, reach_port=None, test_routine=False):

		print("Starting emlid_socketio_client node..")

		rospy.init_node('emlid_socketio_client', anonymous=True, disable_signals=True)

		self.solution_status_publisher = rospy.Publisher("/emlid_solution_status", String, queue_size=1)
		self.gps_stop_publisher = rospy.Publisher("/stop_gps", Bool, queue_size=1)  # stop until GPS gets Fix again

		self.reach_ip = reach_ip or '192.168.131.201'
		self.reach_port = reach_port or 80

		print("Reach IP: {}, Reach Port: {}".format(self.reach_ip, self.reach_port))

		self.reach_keys = [
			'solution status',
			'baseline length float (m)',
			'ratio for ar validation',
			'age of differential (s)'
		]

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

		rospy.sleep(1)

		print("Connecting to SocketIO server from Emlid reach unit..")

		with SocketIO(self.reach_ip, self.reach_port, LoggingNamespace) as socketIO:

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

		if not msg:
			print("No message from Emlid socket.io server.. Sending stop message to rover on /stop_gps..")
			self.gps_stop_publisher.publish(True)
			return

		try:
			solution_status = msg.get('solution status')  # gets solution status from emlid socketio server
		except Exception as e:
			print("Exception: {}".format(e))
			print("Exception getting 'solution status' key from Emlid socketio server.. Sending message to stop rover on /stop_gps..")
			self.gps_stop_publisher.publish(True)
			return

		# self.send_status_to_arduino(solution_status)
		if self.check_for_fix(solution_status):
			self.solution_status_publisher.publish(str(status))  # todo: use json, not just str?
		else:
			self.gps_stop_publisher.publish(True)

			

	# def send_status_to_arduino(self, status):
	# 	"""
	# 	Sends status ('fix', 'float', 'single', or '-') to arduino for
	# 	status light indicator circuit.
	# 	"""
	# 	if status in self.status_options:
	# 		print("Send '{}' message to arduino via {} topic".format(status, '/emlid_solution_status'))
	# 		self.solution_status_publisher.publish(str(status))
	# 	else:
	# 		# raise Exception("Status {} not recognized..")
	# 		print("Status not recognized.. Stopping rover..")
	# 		self.gps_stop_publisher.publish(True)

	# 	return



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

	_reach_ip = None
	_reach_port = None

	try:
		_reach_ip = sys.argv[1]
		_reach_port = sys.argv[2]

	except IndexError:
		print("No inputs provided for reach ip or reach port, so using defaults..")
		print("Trying to use ROS get_param, assuming it's being run as ROS node instead of from terminal..")

		_reach_ip = rospy.get_param('REACH_IP', '192.168.131.201')  # IP address of Reach unit on RoverNet
		_reach_port = rospy.get_param('REACH_PORT', 80)  # connect to Reach HTTP port

	try:
		# Starts Emlid Reach RS SocketIO Client:
		emlidsock = EmlidSocketIOClient(_reach_ip, _reach_port)
	except rospy.ROSInterruptException:
		raise
