"""
Drive patterns for the Jackal or Red Rover to do
when following rows. One pattern, for example, would be
to follow every other row in sequential order until it
reaches the end or all flags are retrieved and it's at the
end of the row.
"""

class DrivePatterns(object):

	def __init__(self):
		self.row_skip = 1  # course row iteration step size (default: 1 - don't skip rows)



	def drive_every_other(self):
		"""
		Drives every other row.
		"""
		self.row_skip = 2
		return self.row_skip



	def drive_to_nearest_flag(self):
		"""
		Determines next row based on nearest
		one that has a flag on it. Has a min row_skip
		setting based on rover's turn radius.
		"""
		pass



	def drive_to_nearest_flag_2(self):
		"""
		Like drive_to_nearest_flag, with the addition
		of checking if dubins path intersects with an
		outer boundary.
		"""
		pass