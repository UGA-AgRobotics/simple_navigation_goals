#!/usr/bin/env python

"""
Python module for reading in flag files and adding any format that's
not already there.

flag file has the following keys: 'date', 'location', 'units', 'flags'.
flags can be in the following units: 'dsm', 'dec', 'utm'.

This module finds the format currently used, and adds the remaining formats to
the flags file.
"""

import utm
import json
import sys
import bag_handler



class FlagFileHandler(object):
	"""
	Handles flag/position data using the keys laid out below.
	See UGA-AgRobotics/RoverWatch/flags for examples of flag files.
	"""

	def __init__(self):

		self.flags = None  # flags json object from flags file

		self.units = ['dsm', 'dec', 'utm']

		self.dsm_object = {
			'deg': None,
			'min': None,
			'sec': None
		}
		self.dsm_pos = {
			'lat': self.dsm_object,
			'lon': self.dsm_object
		}

		self.dec_pos = {
			'lat': None,
			'lon': None
		}

		self.utm_pos = {
			'easting': None,
			'northing': None,
			'zone': None,
			'letter': None
		}

		self.flag_object = {
			'index': None,
			'dsm_pos': None,
			'dec_pos': None,
			'utm_pos': None
		}



	def read_flags_file(self, filename):
		"""
		Reads in flags JSON file.
		"""
		flags_data = None
		with open(filename, 'r') as json_data:
			flags_data = json.loads(json_data.read())

		self.flags = flags_data
		print("flags data set: {}".format(self.flags))



	def save_flags_file(self, filename, updated_flags):
		"""
		Save updated flags file.
		"""
		with open(filename, 'w') as fileout:
			fileout.write(json.dumps(updated_flags))


	def convert_dsm_to_dec(self, dsm_pos):
		"""
		Input lat/lon format: [degree, minute, second]
		"""
		# new_pos = self.dec_pos()
		new_pos = {}
		# new_lat, new_lon = None, None
		dec_pos = None

		for key, val in dsm_pos.items():
			# looping lat/lon key:vals
			if val.get('deg') < 0:
				dec_pos = dsm_pos[key]['deg'] - dsm_pos[key]['min']/60.0 - dsm_pos[key]['sec']/3600.0
			else:
				dec_pos = dsm_pos[key]['deg'] + dsm_pos[key]['min']/60.0 + dsm_pos[key]['sec']/3600.0
			
			new_pos[key] = dec_pos

		return new_pos



	def convert_dec_to_dsm(self, dec_pos):
		"""
		Inputs: lat/lon are single decimal numbers.
		Returns: [lat,lon], where lat/lon = [deg, min, sec]
		"""
		# new_pos = self.dmsPos
		new_pos = {}
		for key, val in dec_pos.items():
			new_pos[key] = {
				'deg': int(val),
				'min': int((val % 1) * 60.0),
				'sec': (((val % 1) * 60.0) % 1) * 60.0
			}
		return new_pos



	def convert_dec_to_utm(self, dec_pos):
		"""
		Converts decimal lat/lon to UTM format.
		"""
		new_pos = self.utm_pos
		utm_val = utm.from_latlon(dec_pos['lat'], dec_pos['lon'])
		new_pos = {
			'easting': utm_val[0],
			'northing': utm_val[1],
			'zone': utm_val[2],
			'letter': utm_val[3]
		}
		return new_pos



	def convert_utm_to_dec(self, utm_pos):
		"""
		Converts UTM position to decimal lat/lon format.
		"""
		new_pos = self.dec_pos
		dec_val = utm.to_latlon(utm_pos['easting'], utm_pos['northing'], utm_pos['zone'], utm_pos['letter'])
		new_pos = {
			'lat': dec_val[0],
			'lon': dec_val[1]
		}
		return new_pos



	def fill_out_flags_file(self):
		"""
		Loops through flags JSON and fills in any formats
		that aren't currently there.
		"""
		if not self.flags:
			raise "No flags specificed. Run read_flags_file(filename) first.."

		if not self.flags.get('units'):
			raise "flags file needs a 'units' key:val of 'dsm', 'dec', or 'utm'"

		flag_date = self.flags.get('date')
		flag_location = self.flags.get('location')
		flag_units = self.flags.get('units')
		flags = self.flags.get('flags')

		if not flags:
			print("STILL NAMED GOALS, TODO: CHANGE NAME TO FLAGS")
			flags = self.flags.get('goals')

		for flag in flags:

			if flag_units == 'dsm':
				# flags file has DSM lat/lons, add dec and utm..
				flag['decPos'] = self.convert_dsm_to_dec(flag['dsmPos'])
				flag['utmPos'] = self.convert_dec_to_utm(flag['decPos'])  # UTM conversion uses dec lat/lons


			elif flag_units == 'dec':
				# flags file has decimal lat/lons, add dsm and utm..
				flag['dsmPos'] = self.convert_dec_to_dsm(flag['decPos'])
				flag['utmPos'] = self.convert_dec_to_utm(flag['decPos'])

			elif flag_units == 'utm':
				# flags file has UTM positions, add dec and dsm..
				flag['decPos'] = self.convert_utm_to_dec(flag['utmPos'])
				flag['dsmPos'] = self.convert_dec_to_dsm(flag['decPos'])

		return flags  # returning updated flags



	def parse_bag_data_to_flags(self):
		"""
		Takes /fix data from bag file handler (e.g., {"topic": "/fix", "data": [{"lat": "", "lon": ""}]}),
		and parses it to the flag JSON format.
		"""
		bag_data_list = self.flags
		parsed_results = {'flags': [], 'units': "dec"}  # assuming /fix topic is dec lat/lon..
		
		if len(bag_data_list) == 1:
			# assuming /fix data is all that's in bag data if length is 1..
			bag_data_list = self.flags[0].get('data')  # get list of {'lat': "", 'lon': ""} objects
		else:
			raise "More than one topic in bag data.. TODO: loop to find desired topic instead of raising exception.."

		_index = 1
		for _pos in bag_data_list:
			parsed_data_obj = {
				'index': _index,
				'decPos': {
					'lat': _pos.get('lat'),
					'lon': _pos.get('lon')
				}
			}
			parsed_results['flags'].append(parsed_data_obj)
			_index += 1

		self.flags = parsed_results  # set flags to parsed results




if __name__ == '__main__':
	"""
	Fills out flags JSON to have all formats of position (UTM, lat/lon deg and dsm).

	Inputs: 
		1. filename - name of initial flags file.
		2. from_bag_file - whether it's position data from a bag file or not (bag_handler.py)

	Output: A parsed JSON file named filename_updated.json
	"""

	filename = None
	from_bag_file = False

	try:
		filename = sys.argv[1]  # get filename from command line
	except IndexError as e:
		raise "Must add an input filename for flags file!"

	try:
		from_bag_file = sys.argv[2]
		if from_bag_file in ["true", "True", True]:
			from_bag_file = True
	except IndexError as e:
		print "Didn't provide bool for data being from bag file or not, assuming it's not from bag_handler.py.."

	flag_file_handler = FlagFileHandler()

	print("flags file read.")
	print("Filling out flags file with missing formats..")

	if from_bag_file:
		print("Assuming GPS topic is /fix in bag file..")
		output_filename = "{}_filled.json".format(filename.split('.bag')[0])  # saves as same filename but w/ .json extension..
		bag_handler.main(filename, output_filename, ["/fix"])  # NOTE: will save output file to path as output_filename..
		print("Filled out GPS data file created: {}".format(output_filename))
		flag_file_handler.read_flags_file(output_filename)  # sets updated/filled data file as flags file..
		flag_file_handler.parse_bag_data_to_flags()  # parses bag_handler data to flags file format before filling out position data..
	else:
		output_filename = "{}_updated.json".format(filename.split('.')[0])
		flag_file_handler.read_flags_file(filename)

	updated_flags = flag_file_handler.fill_out_flags_file()
	flag_file_handler.flags['flags'] = updated_flags

	print("flags file now updated.")
	print("Saving updated flags file as: {}..".format(output_filename))

	flag_file_handler.save_flags_file(output_filename, flag_file_handler.flags)

	print("Done.")