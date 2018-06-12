from socketIO_client import SocketIO, LoggingNamespace
import logging
import serial  # NOTE: This is pySerial library and not serial (i.e., )
import time
import sys



class EmlidSocketIOClient:

	def __init__(self, reach_ip=None, reach_port=None, arduino_path=None, arduino_baud=None):

		# To set DEBUG for more verbose messages for troubleshooting, 
		# uncomment below logging lines:
		# logging.getLogger('socketIO-client').setLevel(logging.DEBUG)
		# logging.basicConfig()

		self.reach_ip = reach_ip or '192.168.131.200'
		self.reach_port = reach_port or 80
		self.arduino_path = arduino_path or '/dev/ttyACM0'
		self.arduino_baud = arduino_baud or 9600

		print("Reach IP: {}, Reach Port: {}".format(self.reach_ip, self.reach_port))
		print("Arduino serial path: {}, Arduino baud: {}".format(self.arduino_path, self.arduino_baud))

		self.reach_keys = [
			'solution status',
			'baseline length float (m)',
			'ratio for ar validation',
			'age of differential (s)'
		]

		self.status_options = ['fix', 'float', 'single', '-']

		self.arduino = serial.Serial(self.arduino_path, self.arduino_baud, timeout=0.2, write_timeout=0.2)  # initiate conn to arduino via serial
		time.sleep(2)  # waiting to ensure connection to arduino before writing to it..

		self.connect_to_socketio_server()  # initiate conn to emlid's socketio server



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

		print("----");
		print("Solution Status: " + solution_status);
		print("Age of Differential (s): " + msg.get('age of differential (s)'));
		print("AR Validation Ratio: " + msg.get('ratio for ar validation'));
		print("Baseline (m): " + msg.get('baseline length float (m)'));
		print("----");

		self.send_status_to_arduino(solution_status)



	def send_status_to_arduino(self, status):
		"""
		Sends status ('fix', 'float', 'single', or '-') to arduino for
		status light indicator circuit.
		"""
		print("status message type: {}".format(type(status)))

		if status in self.status_options:
			print("Send '{}' message to arduino..".format(status))
			self.arduino.write(str(status))
		else:
			raise Exception("Status {} not recognized..")

		return True  # success







if __name__ == '__main__':

	_reach_ip = None
	_reach_port = None
	_arduino_path = None
	_arduino_baud = None

	try:
		_reach_ip = sys.argv[1]
		_reach_port = sys.argv[2]
		_arduino_path = sys.argv[3]
		_arduino_baud = sys.argv[4]
	except IndexError:
		print("No inputs provided for reach ip or reach port, so using defaults..")
		pass

	emlidsock = EmlidSocketIOClient(_reach_ip, _reach_port, _arduino_path, _arduino_baud)