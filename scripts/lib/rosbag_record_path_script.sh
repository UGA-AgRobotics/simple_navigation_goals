#!/bin/bash
# Records rosbag of /fix topic.

rosbag record -O "$1" fix __name:="$2"