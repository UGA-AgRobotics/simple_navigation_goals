#!/usr/bin/env python

# import roslib; roslib.load_manifest('robot_red')
import rospy
import actionlib
import utm

#move_base_msgs
from move_base_msgs.msg import *
from sensor_msgs.msg import NavSatFix
from simple_navigation_goals.srv import *


TestFlagLat = 31.4753776881
TestFlagLon = -83.5289577629
TestFlagUTM = utm.from_latlon(TestFlagLat, TestFlagLon)


def convert_to_utm(lat, lon):
    try:
        return utm.from_latlon(lat, lon)
    except e as Exception:
        print("Error converting lat/lon to utm: {}".format(e))
        return None


def simple_move():

    rospy.init_node('simple_navigation_goals')

    #Simple Action Client
    sac = actionlib.SimpleActionClient('move_base', MoveBaseAction )

    print("Initiating simple_move action..")

    # Get current GPS fix from Jackal's position:
    print("Requestion Jackal's current position via get_jackal_pos service..")
    rospy.wait_for_service('get_jackal_pos')
    get_jackal_pos = rospy.ServiceProxy('get_jackal_pos', JackalPos)
    current_jackal_position = get_jackal_pos()
    print("Jackal's current position: {}".format(current_jackal_position))

    current_jackal_position_utm = convert_to_utm(current_jackal_position.jackal_fix.latitude, current_jackal_position.jackal_fix.longitude)

    print("Jackal's current position in UTM: {}".format(current_jackal_position_utm))

    #create goal
    goal = MoveBaseGoal()

    print("Creating goal...")

    x_diff = abs(TestFlagUTM[0] - current_jackal_position_utm[0])  # magnitude in X direction
    y_diff = abs(TestFlagUTM[1] - current_jackal_position_utm[1])  # magnitude in Y direction

    print("Setting goal with the following x,y scalars: {}m, {}m".format(x_diff, y_diff))

    # #set goal
    # goal.target_pose.pose.position.x = x_diff
    # goal.target_pose.pose.position.y = y_diff
    # goal.target_pose.pose.orientation.w = 0.5
    # goal.target_pose.header.frame_id = 'base_link'
    # goal.target_pose.header.stamp = rospy.Time.now()

    # print("Goal set.")
    # print("Waiting for server...")

    # #start listner
    # sac.wait_for_server()

    # print("Sending goal...")

    # #send goal
    # sac.send_goal(goal)

    # print("Waiting for result...")

    # #finish
    # sac.wait_for_result()

    # #print result
    # print("Result: {}".format(sac.get_result()))

    # # rospy.spin()
   


if __name__ == '__main__':
    try:
        simple_move()
    except rospy.ROSInterruptException:
        print("Keyboard Interrupt")