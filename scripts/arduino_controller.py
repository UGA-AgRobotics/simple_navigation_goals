#!/usr/bin/env python

import serial  # NOTE: This is pySerial library and not serial (i.e., )
import time



class ArduinoController:

	def __init__(self, arduino_serial_port=None, arduino_baud=None):

		self.arduino_serial_port = arduino_serial_port or '/dev/ttyACM2'
		self.arduino_baud = arduino_baud or 9600
		self.arduino = serial.Serial(self.arduino_serial_port, self.arduino_baud, timeout=0.2)  # initiate conn to arduino via serial

		time.sleep(2)  # waiting to ensure connection to arduino before writing to it..



	def simple_arduino_test(self, test_routine):
		"""
		Runs a test routine for arduino serial communication.

		Inputs:
			test_routine - list of messages to send to arduino
		"""

		try:
			print("Running simple_arduino_test!")
			
			for test_message in test_routine:
				print("Sending '{}' to arduino..".format(test_message))
				self.arduino.write(test_message)
				time.sleep(1)
			print("Test complete!")
			return True

		except Exception as e:
			print("Test failed!")
			print("Exception: {}".format(e))
			return False







if __name__ == '__main__':

	arduino_serial_port = None
	arduino_baud = None

	try:
		arduino_serial_port = sys.argv[1]
		arduino_baud = sys.argv[2]
	except IndexError:
		raise IndexError("Must provide arduino_controler with arduino_serial_port and arduino_baud values.")

	ac = ArduinoController(arduino_serial_port, arduino_baud)
	ac.simple_arduino_test()