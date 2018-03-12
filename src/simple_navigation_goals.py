#!/usr/bin/env python

# import roslib; roslib.load_manifest('robot_red')
import rospy
import actionlib

#move_base_msgs
from move_base_msgs.msg import *
from sensor_msgs.msg import NavSatFix
from simple_navigation_goals.srv import *



# def pos_callback(data):
#     """
#     Position callback, which is executed in the event that a GPS fix is
#     published by the Jackal.

#     TODO: Put this subscription in a timer, or refactor in some way to read
#     the GPS values at a slower rate than they're being published.
#     """
#     print("Jackal's lat/lon position: {}, {}".format(data.latitude, data.longitude))


def simple_move():

    rospy.init_node('simple_navigation_goals')

    #Simple Action Client
    sac = actionlib.SimpleActionClient('move_base', MoveBaseAction )

    print("Initiating simple_move action..")

    # # Subscribe to Jackal's /navsat/fix topic:
    # # jackal_pos_subscriber()
    # rospy.Subscriber('/navsat/fix', NavSatFix, pos_callback)

    # Get current GPS fix from Jackal's position:
    print("Requestion Jackal's current position via get_jackal_pos service..")
    rospy.wait_for_service('get_jackal_pos')
    get_jackal_pos = rospy.ServiceProxy('get_jackal_pos', JackalPos)
    current_jackal_position = get_jackal_pos()
    print("Jackal's current position: {}".format(current_jackal_position))

    #create goal
    goal = MoveBaseGoal()

    print("Creating goal...")

    # #use self?
    # #set goal
    # goal.target_pose.pose.position.x = 0.5
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
    # print sac.get_result()

    # rospy.spin()
   


if __name__ == '__main__':
    try:
        simple_move()
    except rospy.ROSInterruptException:
        print("Keyboard Interrupt")