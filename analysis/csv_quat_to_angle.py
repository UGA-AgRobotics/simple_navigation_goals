# import roslib
# import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from math import radians, copysign, sqrt, pow, pi
import PyKDL
import sys
import numpy as np
import csv



def quat_to_angle(quatx, quaty, quatz, quatw):
    """
    Converts quaternion to angle.
    """
    rot = PyKDL.Rotation.Quaternion(quatx, quaty, quatz, quatw)
    return rot.GetRPY()[2]


if __name__ == '__main__':

	csv_file = sys.argv[1]

	_angles = []  # list of angles from Quaternion values
	with open(csv_file, 'r') as csv_file:
		csvreader = csv.reader(csv_file)
		for row in csvreader:
			# _quat_obj = np.array([])
			_angle = quat_to_angle(float(row[0]), float(row[1]), float(row[2]), float(row[3]))
			print("Converted angle: {}".format(_angle))
			_angles.append(_angle)

	fileout = open('data_out.csv', 'w')
	for item in _angles:
		fileout.write(str(item) + '\n')
	fileout.close()

	# with open('data_out.csv', 'w') as fileout:
	# 	csvwriter = csv.writer(fileout)
	# 	for row in _angles:
	# 		csvwriter.writerow(row)

