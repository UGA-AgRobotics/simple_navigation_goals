This is the beginning of creating a robot-agnostic
setup for driving down a path of GPS points. Basically, the
drive module would read in IMU, GPS, and path data, and
each robot would have a module that tranlates the drive
node's general commands (e.g., turn, drive, stop).