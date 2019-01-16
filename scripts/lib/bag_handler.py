"""
Rosbag API Example from http://wiki.ros.org/rosbag/Code%20API

Goal: Function, module, whatever makes sense, that the frontend can call
to initiate the ros bag recording, start and stop.
"""


import sys
import rosbag
import math
import json
import utm
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Twist, Point, Quaternion
import PyKDL





class BagHandler:

	def __init__(self, filename, topics):
		self.filename = filename  # bag file name (name for recording bag file, or name to read data from bag file)
		self.topics = topics  # list of topics to record or read from the bag file

		self.imu_topic = '/phidget/imu/data'
		self.reach_topic = '/fix'

		self.imu_result_obj = {'angle': None}
		self.reach_result_obj = {'lat': None, 'lon': None}

		self.data_from_bag = {}  # the parsed/massaged data from bagfile that'll be used as a course/navigation file for the Rover


	def get_data_from_bag(self):
		"""
		Loops through bag data grabbing specific keys and values..
		"""
		_bag = rosbag.Bag(self.filename)  # get bag file object
		# _bag_results = {}  # parsed bag results
		_bag_results_list = []  # list of bag results

		print("rosbag: {}".format(_bag))

		for topic in self.topics:
			_topic_results = {
				'topic': topic,
				'data': []
			}
			_bag_results_list.append(_topic_results)


		for topic, msg, t in _bag.read_messages(topics=self.topics):

			if topic == self.imu_topic:
				_data_obj = self.handle_imu_data(msg)

			elif topic == self.reach_topic:
				_data_obj = self.handle_reach_data(msg)

			_bag_results_list = self.add_topic_data_to_results(topic, _bag_results_list, _data_obj)

		_bag.close()


		print "Data from {} topics has been successfully retrieved from bagfile {}".format(self.topics, self.filename)
		print "Data can now be accessed by the 'data_from_bag' attribute.."

		self.data_from_bag = _bag_results_list
		return _bag_results_list


	def save_data_from_bag(self, output_filename):
		"""
		Saves parsed data from bagfile. Used after running
		get_data_from_bag() function..
		"""
		fileout = open(output_filename, 'w')
		fileout.write(json.dumps(self.data_from_bag))
		fileout.close()
		print "Data from bagfile saved: {}".format(output_filename)
		return


	def handle_imu_data(self, msg):
		"""
		Grabs orientation data from IMU and converts it to an angle value.
		"""
		angle_rad = self.quat_to_angle(msg.orientation)  # convert orientation from quaternion to angle (radians)
		# imu_result = self.imu_result_obj
		imu_result = {}
		imu_result['angle'] = math.degrees(angle_rad)
		return imu_result
		

	def handle_reach_data(self, msg):
		"""
		Grabs GPS data from Reach unit lat/lon positions.
		"""
		# reach_result = self.reach_result_obj
		reach_result = {}
		reach_result['lat'] = msg.latitude
		reach_result['lon'] = msg.longitude
		return reach_result


	def quat_to_angle(self, quat):
		"""
		Converts quaternion to angle.
		"""
		rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
		return rot.GetRPY()[2]


	def add_topic_data_to_results(self, topic, bag_results_list, data_obj):
		"""
		Loops through parsed bag results and adds data to
		the appropriate topic.
		"""
		for bag_result_obj in bag_results_list:
			if bag_result_obj.get('topic') == topic:
				bag_result_obj.get('data', []).append(data_obj)  # add data to topic in results list
		return bag_results_list



	def get_path_array_from_rosbag(self, fix_topic="/fix"):
		"""
		Gets /fix data from rosbag and returns an list
		of [easting, northing] pairs.
		"""
		_bag = rosbag.Bag(self.filename)

		path_array = []
		for topic, msg, t in _bag.read_messages(topics=self.topics):
			if topic == fix_topic:
				utm_pos = utm.from_latlon(float(msg.latitude), float(msg.longitude))
				pos_array = [utm_pos[0], utm_pos[1]]
				path_array.append(pos_array)
		return path_array



def main(bagfilename, output_filename, topic_list):
	print("Getting {} topics data from {} bagfile".format(topic_list, bagfilename))
	bagobj = BagHandler(bagfilename, topic_list)
	bagobj.get_data_from_bag()
	bagobj.save_data_from_bag(output_filename)
	print("Data saved as: {}".format(output_filename))
		






if __name__ == '__main__':
	"""
	Records a ros bag of topics based on the following args when running
	this module on the terminal:

	e.g., python record_course.py rosbagfile1.bag /fix /imu/data <- record /fix
	and /imu/data topics into a file called rosbagfile1.bag
	"""
	# Example inputs:
	######################################
	# bagfilename = sample_course_1.bag
	# output_filename = course_1.json
	# topic_list = /fix
	######################################

	bagfilename = None
	output_filename = None
	topic_list = []

	try:
		bagfilename = sys.argv[1]
		output_filename = sys.argv[2]

		for topic in sys.argv[3:]:
			topic_list.append(topic)  # NOTE: assuming rest of args are topics
	except IndexError:
		raise IndexError("Must have the following args: bagfilename, output filename, and topics")
	
	main(bagfilename, output_filename, topic_list)  # Runs main bag_handler routine!