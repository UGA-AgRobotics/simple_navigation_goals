#!/bin/bash
# Records rosbag of /fix topic.

rosbag record -O "../../courses/$1" /fix __name:="$2"