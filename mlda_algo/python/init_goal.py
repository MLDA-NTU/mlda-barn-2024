#!/usr/bin/python3

import rospy
import actionlib
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

rospy.init_node("init_goal", anonymous=True)
nav_as = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
mb_goal = MoveBaseGoal()
mb_goal.target_pose.header.frame_id = "odom"
mb_goal.target_pose.pose.position.x = 10  # With respect to odom frame
mb_goal.target_pose.pose.position.y = 10
mb_goal.target_pose.pose.position.z = 0
mb_goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)

nav_as.wait_for_server()
nav_as.send_goal(mb_goal)
