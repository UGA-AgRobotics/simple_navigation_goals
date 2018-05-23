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


	# def record_bagfile(self):
	# 	_bag = rosbag.Bag('test.bag', 'w')
	# 	try:
	# 		str = String()
	# 		str.data = 'foo'
	# 		i = Int32()
	# 		i.data = 42
	# 		_bag.write('chatter', str)
	# 		_bag.write('numbers', i)
	# 	finally:
	# 		_bag.close()


	def get_data_from_bag(self):
		"""
		Loops through bag data grabbing specific keys and values..
		"""
		_bag = rosbag.Bag(self.filename)
		_bag_results = {}

		# Creates keys using the topic names for _bag_results:
		for topic in self.topics:
			_bag_results[topic] = []

		for topic, msg, t in _bag.read_messages(topics=self.topics):

			if topic == self.imu_topic:
				_data_obj = self.handle_imu_data(msg)
				_bag_results[topic].append(_data_obj)

			elif topic == self.reach_topic:
				_data_obj = self.handle_reach_data(msg)
				_bag_results[topic].append(_data_obj)

		_bag.close()


		print "Data from {} topics has been successfully retrieved from bagfile {}".format(self.topics, self.filename)
		print "Data can now be accessed by the 'data_from_bag' attribute.."

		self.data_from_bag = _bag_results
		return _bag_results


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







if __name__ == '__main__':
	"""
	Records a ros bag of topics based on the following args when running
	this module on the terminal:

	e.g., python record_course.py rosbagfile1.bag /fix /imu/data <- record /fix
	and /imu/data topics into a file called rosbagfile1.bag
	"""
	bagfilename = 'sample_course_1.bag'
	output_filename = 'course_1.json'
	topic_list = ['/fix']
	
	# Testing routine:
	print "Getting {} topics data from {} bagfile".format(topic_list, bagfilename)
	bagobj = BagHandler(bagfilename, topic_list)
	bagobj.get_data_from_bag()
	bagobj.save_data_from_bag(output_filename)
	print "Data saved as: {}".format(output_filename)


	# Code block for running bag recording class:
	# try:
	# 	# SingleGoalNav()
	# 	filename = sys.argv[1]
	# 	topics = []
	# 	for topic in sys.argv[2:]:
	# 		topics.append(topic)  # assuming remaining args are topics
	# 	# TODO: Finish routine..
	# except rospy.ROSInterruptException:
	# 	rospy.loginfo("Navigation terminated.")

