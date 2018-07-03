import sys
import csv



def convert_emlid_logfile_to_latlons(input_filename, output_filename):

	filein = open(filename, 'r')
	data = filein.read()  # reads in file to object
	filein.close()

	data_list = data.split('\n')  # gets each row of data
	data_arary = []

	for data_row in data_list:
		data_row_list = data_row.split('  ')
		for item in data_row_list:
			if len(item) > 0:
				data_arary.append([data_row_list[1], data_row_list[2]])  # grabs lat/lons

	fileout = open(parsedfilename, 'wb')
	csvwriter = csv.writer(fileout)
	csvwriter.writerows(data_arary)
	fileout.close()



def convert_emlid_logfile_to_course(input_filename, output_filename):

	



if __name__ == '__main__':

	filename = sys.argv[1]  # gets filename of .LLH file from emlid
	parsedfilename = sys.argv[2]  # gets filename to save parsed data

	convert_emlid_logfile_to_latlons(filename, parsedfilename)