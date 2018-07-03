import sys
import csv
import json



class LogfileParser:

	def __init__(self):
		pass



	def open_input_file(self, filename):

		filein = open(filename, 'r')
		data = filein.read()  # reads in file to object
		filein.close()
		return data



	def write_to_output_csv(self, filename, csv_data):

		fileout = open(filename, 'wb')
		csvwriter = csv.writer(fileout)
		csvwriter.writerows(csv_data)
		fileout.close()



	def write_to_output_json(self, filename, json_data):

		fileout = open(filename, 'w')
		fileout.write(json.dumps(json_data))
		fileout.close()



	def convert_emlid_logfile_to_latlons(self, input_filename, output_filename):

		data_from_file = self.open_input_file(input_filename)

		data_list = data_from_file.split('\n')  # gets each row of data
		data_arrary = []  # list of list for csv output

		for data_row in data_list:
			data_row_list = data_row.split('  ')
			for item in data_row_list:
				if len(item) > 0:
					data_arrary.append([data_row_list[1], data_row_list[2]])  # grabs lat/lons

		self.write_to_output_csv(output_filename, data_arrary)



	def convert_emlid_logfile_to_course(self, input_filename, output_filename, date=None, location=None):

		n_skip = 1  # grab every nth point (default: 1)

		data_json = {}
		data_json['goals'] = []
		data_json['date'] = date
		data_json['units'] = "dec"
		data_json['location'] = location


		data_from_file = self.open_input_file(input_filename)

		data_list = data_from_file.split('\n')  # gets each row of data

		ind = 1  # index counter

		print("Getting every {}th row..".format(n_skip))

		for i in range(0, len(data_list) - 1, n_skip):

			data_row_list = data_list[i].split('  ')

			_time = None  # get timestamp of position datum

			for item in data_row_list:
				if len(item) > 0:
					decPos =  {
						'lat': float(data_row_list[1]),
						'lon': float(data_row_list[2])
					}
					_time = data_row_list[0]

			goal_obj = {
				'index': ind,
				'dsmPos': {},
				'utmPos': {},
				'decPos': decPos,
				'time': _time
			}

			data_json['goals'].append(goal_obj)

			ind += 1

		self.write_to_output_json(output_filename, data_json)



	def convert_emlid_logfile_to_geojson(self, input_filename, output_filename):

		n_skip = 1

		data_json = {}
		data_json['type'] = "FeatureCollection"
		data_json['features'] = []

		data_from_file = self.open_input_file(input_filename)
		data_list = data_from_file.split('\n')  # gets each row of data

		print("Getting every {}th data point".format(n_skip))

		for i in range(0, len(data_list) - 1, n_skip):

			data_row_list = data_list[i].split('  ')

			_lat, _lon = None, None

			for item in data_row_list:
				if len(item) > 0:
					_lat = float(data_row_list[1])
					_lon = float(data_row_list[2])

			feature_obj = {
				'type': "Feature",
				'geometry': {
					'type': "Point",
					'coordinates': [_lat, _lon]
				}
			}

			data_json['features'].append(feature_obj)

		self.write_to_output_json(output_filename, data_json)

		






	



if __name__ == '__main__':

	filename = sys.argv[1]  # gets filename of .LLH file from emlid
	parsedfilename = sys.argv[2]  # gets filename to save parsed data
	date = None
	location = None

	try:
		date = sys.argv[3]
		location = sys.argv[4]
	except IndexError:
		raise


	lp = LogfileParser()

	# Parses emlid log into file of lat,lons:
	# lp.convert_emlid_logfile_to_latlons(filename, parsedfilename)
	
	# Parses emlid log into course file format:
	# lp.convert_emlid_logfile_to_course(filename, parsedfilename, date, location)

	# Parses emlid log into geojson file format:
	lp.convert_emlid_logfile_to_geojson(filename, parsedfilename)