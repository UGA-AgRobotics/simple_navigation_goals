#!/usr/bin/env python

import os
import rospy
import sys
import logging
# from dotenv import load_dotenv
from io import StringIO
from dotenv import dotenv_values
import yaml
import json



logging.info("red_rover_config.py")

class DeployEnv(object):
	"""
	Loads environment variables for ROS
	"""

	def __init__(self, yaml_file=None, json_file=None):
		self.yaml_file = yaml_file or 'red_rover_config.yaml'  # filename for .yaml environment
		self.yaml_env = {}  # dict for .yaml environment



	def load_yaml_deployment_environment(self):
		"""
		Loads the yaml env file instead a python dictionary
		"""
		with open(self.yaml_file, 'r') as stream:
			try:
				self.yaml_env = yaml.safe_load(stream)
			except yaml.YAMLError as e:
				print("Error reading yaml env var file at config/red_rover_config.py")
				raise e



	# def load_json_environment_file(self):
	# 	"""
	# 	Same as above load_deployment_environment(), but for json file.
	# 	Going to use yaml unless objects become an issue (i.e., serializing 
	# 	list and dicts from yaml file)
	# 	"""
	# 	with open(self.json_file, 'r') as stream:
	# 		try:
	# 			self.json_env = json.loads(stream.read())
	# 		except Exception:
	# 			print("Error loading env vars from .json file: {}".format(self.json_file))
	# 			raise



	def add_vars_to_rosparams(self):
		"""
		Loops through key:vals from yaml env file and sets them as
		rosparams.
		"""
		print("Env vars from {}:".format(self.yaml_file))
		for key, val in self.yaml_env.items():
			
			print("{} = {}".format(key, val))
			rospy.set_param(key, str(val))

		print("rosparams set!")



	# def add_vars_to_environment(self):
	# 	"""
	# 	Uses dotenv python module to store yaml params in the environment.
	# 	Using a .yaml instead of .env in case we migrate everything to just using
	# 	ROS (i.e., all config in rosparam server), so env vars are set
	# 	using file-likes (https://github.com/theskumar/python-dotenv#in-memory-filelikes)
	# 	"""

	# 	for key, val in self.yaml_env.items():

	# 		filelike = StringIO("{}={}\n".format(key, val))  # key:val stored in-memory temporarily
	# 		filelike.seek(0)  # set "file's" current position to beginning (rewind before passing)
	# 		parsed = dotenv_values(stream=filelike)
			





if __name__ == '__main__':

	deploy_env = DeployEnv()
	print("Loading .yaml file: {}".format(deploy_env.yaml_file))
	deploy_env.load_yaml_deployment_environment()
	print("success.")
	print("Adding config as rosparams..")
	deploy_env.add_vars_to_rosparams()
	print("success.")
	# print("Adding config to environment as well..")
	# deploy_env.add_vars_to_environment()
	# print("success.")