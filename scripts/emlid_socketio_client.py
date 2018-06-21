#!/usr/bin/env python

from socketIO_client import SocketIO, LoggingNamespace
import logging
import serial  # NOTE: This is pySerial library and not serial (i.e., )
import time
import sys
import roslib
import rospy
from std_msgs.msg import String
from requests.exceptions import ConnectionError
# from arduino_controller import ArduinoController



class EmlidSocketIOClient:

	def __init__(self, reach_ip=None, reach_port=None, arduino_serial_port=None, arduino_baud=None, test_routine=False):

		print("Starting emlid_socketio_client node..")

		rospy.init_node('emlid_socketio_client', anonymous=True, disable_signals=True)

		self.solution_status_publisher = rospy.Publisher("/emlid_solution_status", String, queue_size=1)

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
		print("Shutting down socketio client ROS node..")
		rospy.signal_shutdown("Socketio server disconnected, shutting down emlid_socketio_client ROS node.")
		print('disconnected.')



	def on_reconnect(self):
		print('reconnected.')



	def on_status_broadcast(self, msg):

		if not msg: return
		
		solution_status = msg.get('solution status')
		self.send_status_to_arduino(solution_status)



	def send_status_to_arduino(self, status):
		"""
		Sends status ('fix', 'float', 'single', or '-') to arduino for
		status light indicator circuit.
		"""
		if status in self.status_options:
			print("Send '{}' message to arduino via {} topic".format(status, '/emlid_solution_status'))
			self.solution_status_publisher.publish(str(status))
		else:
			raise Exception("Status {} not recognized..")

		return







if __name__ == '__main__':

	_reach_ip = None
	_reach_port = None
	_arduino_serial_port = None
	_arduino_baud = None

	try:
		_reach_ip = sys.argv[1]
		_reach_port = sys.argv[2]
		_arduino_serial_port = sys.argv[3]
		_arduino_baud = sys.argv[4]

	except IndexError:
		print("No inputs provided for reach ip or reach port, so using defaults..")
		print("Trying to use ROS get_param, assuming it's being run as ROS node instead of from terminal..")

		_reach_ip = rospy.get_param('REACH_IP', '192.168.131.201')  # IP address of Reach unit on RoverNet
		_reach_port = rospy.get_param('REACH_PORT', 80)  # connect to Reach HTTP port
		_arduino_serial_port = rospy.get_param('ARDUINO_SERIAL_PORT', '/dev/ttyACM2')  # tty port for Arduino
		_arduino_baud = rospy.get_param('ARDUINO_BAUD', 9600)  # baud rate for arduino serial communication

	try:
		# Starts Emlid Reach RS SocketIO Client:
		emlidsock = EmlidSocketIOClient(_reach_ip, _reach_port, _arduino_serial_port, _arduino_baud)
	except rospy.ROSInterruptException:
		raise
