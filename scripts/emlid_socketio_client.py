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



class EmlidSocketIOParameters:
	"""
	This class lays out the Emlid's SocketIO server
	settings that are relevant to the rover's navigation.
	"""

	def __init__(self, emlid_ip=None, emlid_port=None):

		print("Setting emlid socketio server parameters.")

		# IP address and port for emlid socketio server:
		self.emlid_ip = emlid_ip or '192.168.131.201'
		self.emlid_port = emlid_port or 80

		# Emlid GPS status options:
		self.status_options = ['fix', 'float', 'single', '-']

		# SocketIO server response keys for GPS:
		self.response_keys = ['receiver time mark count', 'ant type base', 'solution status', 'vel enu (m/s) base', 'time of receiver clock rover', 'baseline length float (m)', '# of rtcm messages rover', 'time sys offset (ns)', '# of average single pos base', 'pos xyz float std (m) rover', 'ant delta rover', '# of rtcm messages corr', 'accumulated time to run', 'cpu time for a cycle (ms)', '# of rtcm messages base', 'vel enu (m/s) rover', 'ant type rover', 'pos xyz single (m) rover', '\x1b[1mParameter', 'rtk server state', 'pos xyz fixed (m) rover', 'age of differential (s)', 'pos xyz fixed std (m) rover', '# of all estimated states', 'processing cycle (ms)', 'ant delta base', '# of valid satellites', 'rtklib version', 'bytes in input buffer', '# of input data corr', '# of input data rover', 'pos xyz (m) base', 'ratio for ar validation', 'pos llh (deg,m) base', 'missing obs data count', '# of real estimated states', 'baseline length fixed (m)', '# of satellites base', 'last time mark', '# of input data base', '# of satellites rover', 'GDOP/PDOP/HDOP/VDOP', 'pos xyz float (m) rover', 'positioning mode', 'pos llh single (deg,m) rover', 'solution interval (s)', 'rtklib time mark count']
		
		# Example response object from emlid's socketio server:
		self.response_example = {"receiver time mark count": "0", "ant type base": "", "solution status": "fix", "vel enu (m/s) base": "0.000,0.000,0.000", "time of receiver clock rover": "2018/09/05 18:07:34.800360200", "baseline length float (m)": "3.840", "# of rtcm messages rover": "", "time sys offset (ns)": "0.000,0.000,0.000,0.000", "# of average single pos base": "0", "pos xyz float std (m) rover": "0.006,0.012,0.009", "ant delta rover": "0.000 0.000 0.000", "# of rtcm messages corr": "", "accumulated time to run": "01:29:22.2", "cpu time for a cycle (ms)": "28", "# of rtcm messages base": "1004(5355),1005(1071),1007(1071),1012(5355),1019(536),1030(1071),1031(1071),1032(536),1033(1071),other3(1072)", "vel enu (m/s) rover": "-0.052,0.004,-0.053", "ant type rover": "", "pos xyz single (m) rover": "660552.040,-5173953.727,3659529.854", "\x1b[1mParameter": "Value\x1b[0m", "rtk server state": "run", "pos xyz fixed (m) rover": "660552.040,-5173953.727,3659529.854", "age of differential (s)": "0.793", "pos xyz fixed std (m) rover": "0.006,0.012,0.009", "# of all estimated states": "170", "processing cycle (ms)": "10", "ant delta base": "0.000 0.000 0.000", "# of valid satellites": "7", "rtklib version": "2.4.3 Emlid b28", "bytes in input buffer": "0,0", "# of input data corr": "obs(0),nav(0),gnav(0),ion(0),sbs(0),pos(0),dgps(0),ssr(0),err(0)", "# of input data rover": "obs(26805),nav(20),gnav(0),ion(1479),sbs(0),pos(0),dgps(0),ssr(0),err(0)", "pos xyz (m) base": "660552.459,-5173950.029,3659528.885", "ratio for ar validation": "46.156", "pos llh (deg,m) base": "35.23478873,-82.72446983,615.833", "missing obs data count": "0", "# of real estimated states": "9", "baseline length fixed (m)": "3.846", "# of satellites base": "13", "last time mark": "-", "# of input data base": "obs(5355),nav(19),gnav(0),ion(0),sbs(0),pos(3213),dgps(0),ssr(0),err(0)", "# of satellites rover": "9", "GDOP/PDOP/HDOP/VDOP": "2.3,2.0,1.1,1.7", "pos xyz float (m) rover": "660552.039,-5173953.721,3659529.851", "positioning mode": "kinematic", "pos llh single (deg,m) rover": "35.23477707,-82.72447955,619.345", "solution interval (s)": "0.200", "rtklib time mark count": "0"}

		# Defined variables for emlid keys that are relevant to the rover's GPS:
		self.rover_pos_key = 'pos llh single (deg,m) rover'
		self.rover_status_key = 'solution status'
		self.rover_ar_ratio = 'ratio for ar validation'
		self.rover_baseline = 'baseline length float (m)'



class EmlidSocketIOClient(EmlidSocketIOParameters):
	"""
	This class acts as a client to the Emlid's SocketIO
	server. It connects to the Emlid unit's SocketIO server,
	and broadcasts its info as ROS topics.

	Publishers:
		/fix - (NavSatFix) The GPS position.
		/emlid_info - The JSON data coming from the Emlid.
		/stop_gps - (bool) Stops rover if (sends True) if GPS isn't a fix.
	"""

	def __init__(self, emlid_ip=None, emlid_port=None, test_routine=False):

		EmlidSocketIOParameters.__init__(self, emlid_ip, emlid_port)

		print("Starting emlid_socketio_client node.")

		rospy.init_node('emlid_socketio_client', anonymous=True, disable_signals=True)

		self.is_connected = False  # emlid socketio server connection bool
		self.msg_num = 0  # incoming message number/index since connection

		# Subscribers:

		# Publishers:
		self.solution_status_publisher = rospy.Publisher("/emlid_solution_status", String, queue_size=1)
		self.gps_stop_publisher = rospy.Publisher("/stop_gps", Bool, queue_size=1)  # stop until GPS gets Fix again
		self.gps_pos_publisher = rospy.Publiser("/fix", NavSatFix, queue_size=1)

		rospy.sleep(2)

		print("Connecting to SocketIO server from Emlid reach unit.")

		with SocketIO(self.emlid_ip, self.emlid_port, LoggingNamespace) as socketIO:

			socketIO.on('connect', self.on_connect)
			socketIO.on('disconnect', self.on_disconnect)
			socketIO.on('reconnect', self.on_reconnect)
			socketIO.on('status broadcast', self.on_status_broadcast)

			rospy.sleep(1)

			print("emlid_socketio_client node ready.")

			socketIO.wait()



	def on_connect(self):
		print("Connected to rover's Emlid Reach SocketIO server.")
		print("Reach IP: {}, Reach Port: {}".format(self.emlid_ip, self.emlid_port))
		self.is_connected = True
		self.msg_num = 0



	def on_disconnect(self):
		print("Emlid node's socketio connection disconnected. Sending message to stop rover on /stop_gps.")
		self.gps_stop_publisher.publish(True)

		self.is_connected = False
		self.msg_num = 0

		print("Shutting down socketio client ROS node.")
		rospy.signal_shutdown("Socketio server disconnected, shutting down emlid_socketio_client ROS node.")
		
		print('disconnected.')



	def on_reconnect(self):
		print('reconnected.')
		self.is_connected = True
		self.msg_num = 0



	def on_status_broadcast(self, msg):

		try:
			emlid_data = json.loads(json.dumps(msg))  # re-serialize for the safety (ensures it's JSON-safe data coming in)
		except Exception as e:
			print("Could not reserialize incoming JSON from emlid socketio server.")
			return

		if not msg:
			print("No message from Emlid socket.io server. Sending stop message to rover on /stop_gps.")
			self.gps_stop_publisher.publish(True)
			return

		print("Message {}:".format(self.msg_num))
		print("GPS Position: {}".format(emlid_data[self.rover_pos_key]))
		print("GPS Status: {}".format(emlid_data[self.rover_status_key]))
		print("GPS AR Validation Ratio: {}".format(emlid_data[self.rover_ar_ratio]))
		self.msg_num += 1

		self.publish_emlid_pos(emlid_data)



	def publish_emlid_pos(self, emlid_data):
		"""
		Parses the lat,lon,alt string into a NavSatFix object, 
		and publishes to /fix topic.
		"""

		# Gets LLH values:
		emlid_list = emlid_data.split(',')
		emlid_lat = float(emlid_list[0])
		emlid_lon = float(emlid_list[1])
		emlid_alt = float(emlid_list[2])

		# Creates NavSatFix object:
		navsat_pos = NavSatFix()
		navsat_pos['latitude'] = emlid_lat
		navsat_pos['longitude'] = emlid_lon
		navsat_pos['altitude'] = emlid_alt

		print("Publishing to /fix: {}".format(navsat_pos))

		# Publishes NavSatFix value to /fix topic:
		self.gps_pos_publisher.publish(navsat_pos)



	def check_for_fix(self, status):
		"""
		Sends True on /stop_fix topic if Emlid has anything
		other than a fix.
		"""
		if not status in self.status_options:
			print("Status not recognized. Stopping rover.")
			self.gps_stop_publisher.publish(True)
			return False

		if status == 'fix':
			self.gps_stop_publisher.publish(False)  # starts rover after getting fix
			return True

		else:
			print(">>> Emlid node sending message on /stop_gps to stop rover due to losing GPS fix.")
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
