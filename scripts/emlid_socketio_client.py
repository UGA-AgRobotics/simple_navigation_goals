#!/usr/bin/env python

from socketIO_client import SocketIO, LoggingNamespace
import logging
import serial  # NOTE: This is pySerial library and not serial (i.e., )
import time
import sys
import roslib
import rospy
from arduino_controller import ArduinoController



class EmlidSocketIOClient:

	def __init__(self, reach_ip=None, reach_port=None, arduino_serial_port=None, arduino_baud=None, test_routine=False):

		# To set DEBUG for more verbose messages for troubleshooting, 
		# uncomment below logging lines:
		# logging.getLogger('socketIO-client').setLevel(logging.DEBUG)
		# logging.basicConfig()

		self.reach_ip = reach_ip or '192.168.131.201'
		self.reach_port = reach_port or 80
		self.arduino_serial_port = arduino_serial_port or '/dev/ttyACM2'
		self.arduino_baud = arduino_baud or 9600

		print("Reach IP: {}, Reach Port: {}".format(self.reach_ip, self.reach_port))
		print("Arduino serial path: {}, Arduino baud: {}".format(self.arduino_serial_port, self.arduino_baud))

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

		self.arduino_controller = ArduinoController(self.arduino_serial_port, self.arduino_baud)

		# Runs socketio server if not running a test routine:
		if test_routine:
			self.run_light_test()
		else:
			self.connect_to_socketio_server()  # initiate connection to emlid's socketio server



	def run_light_test(self):
		"""
		Tests signal lights on arduino for solution status, et al.
		"""
		test_messages = self.status_options + [self.off_signal, self.flag_on_signal, self.flag_off_signal]
		print("Sending the following test messages to arduino test routine: {}".format(test_messages))
		test_result = self.arduino_controller.simple_arduino_test(test_messages)
		return test_result



	def connect_to_socketio_server(self):

		with SocketIO(self.reach_ip, self.reach_port, LoggingNamespace) as socketIO:

			socketIO.on('connect', self.on_connect)
			socketIO.on('disconnect', self.on_disconnect)
			socketIO.on('reconnect', self.on_reconnect)
			socketIO.on('status broadcast', self.on_status_broadcast)

			socketIO.wait()



	def on_connect(self):
		print('connected.')



	def on_disconnect(self):
		print('disconnected.')



	def on_reconnect(self):
		print('reconnected.')



	def on_status_broadcast(self, msg):

		if not msg: return
		
		solution_status = msg.get('solution status')

		#print("----");
		#print("Solution Status: " + solution_status);
		#print("Age of Differential (s): " + msg.get('age of differential (s)'));
		#print("AR Validation Ratio: " + msg.get('ratio for ar validation'));
		#print("Baseline (m): " + msg.get('baseline length float (m)'));
		#print("----");

		self.send_status_to_arduino(solution_status)



	def send_status_to_arduino(self, status):
		"""
		Sends status ('fix', 'float', 'single', or '-') to arduino for
		status light indicator circuit.
		"""
		# print("status message type: {}".format(type(status)))

		if status in self.status_options:
			# print("Send '{}' message to arduino..".format(status))
			# self.arduino.write(str(status))
			self.arduino_controller.arduino.write(str(status))
		else:
			raise Exception("Status {} not recognized..")

		return True  # success







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

	# Starts Emlid Reach RS SocketIO Client:
	emlidsock = EmlidSocketIOClient(_reach_ip, _reach_port, _arduino_serial_port, _arduino_baud)
