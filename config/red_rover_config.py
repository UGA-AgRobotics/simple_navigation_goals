#!/usr/bin/env python

import os
import rospy
import sys
import logging
import dotenv
#from dotenv import load_dotenv
from io import StringIO
#from dotenv import dotenv_values
import yaml
import json



logging.info("red_rover_config.py")

class DeployEnv(object):
	"""
	Loads environment variables for ROS
	"""

	def __init__(self, yaml_file=None, env_file=None):
		self.yaml_file = yaml_file or 'red_rover_config.yaml'  # filename for .yaml environment
		self.yaml_env = {}  # dict for .yaml environment
		self.env_file = env_file or '.env'



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
	# 	# print("Loading env vars from {} to the environment..".format(self.env_file))
	# 	# dotenv.load_dotenv(dotenv_path='red_rover_config.env')

	# 	# for key, val in self.yaml_env.items():
	# 	# 	filelike = StringIO("{}={}\n".format(key, val))  # key:val stored in-memory temporarily
	# 	# 	filelike.seek(0)  # set "file's" current position to beginning (rewind before passing)
	# 	# 	parsed = dotenv_values(stream=filelike)
	# 	# print("Env vars note: everything is considered a string")
	# 	# for key, val in self.yaml_env.items():
	# 	# 	dotenv.set_key(self.env_file, key, str(val))
	# 	dotenv.load_dotenv(self.env_file)



if __name__ == '__main__':

	deploy_env = DeployEnv()
	print("Loading .yaml file: {}".format(deploy_env.yaml_file))
	deploy_env.load_yaml_deployment_environment()
	print("success.")
	print("Adding config as rosparams..")
	deploy_env.add_vars_to_rosparams()
	print("success.")
	# print("Setting env vars from .yaml file.. Saving as {} in config/".format(deploy_env.env_file))
	# deploy_env.add_vars_to_environment()
	# print("success.")
