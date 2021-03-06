#!/usr/bin/env python
import rospy
import actionlib
from fetch_open_auto_dock.msg import DockAction, DockGoal

# Create a ROS node
rospy.init_node("dock_the_robot")

# Create an action client
client = actionlib.SimpleActionClient("/dock", DockAction)
client.wait_for_server()
print("server started!")
# Create and send a goal
goal = DockGoal()
#goal.dock_pose.header.frame_id = "base_link"
#goal.dock_pose.pose.position.x = 1.5
#goal.dock_pose.pose.orientation.x = 0.0
#goal.dock_pose.pose.orientation.y = 0.0
#goal.dock_pose.pose.orientation.z = 0.0
#goal.dock_pose.pose.orientation.w = 0.0

goal.dock_pose.header.frame_id = "map"
goal.dock_pose.pose.position.x = 0
goal.dock_pose.pose.position.y = -2
goal.dock_pose.pose.position.z = 0
goal.dock_pose.pose.orientation.x = 0.0
goal.dock_pose.pose.orientation.y = 0.0
goal.dock_pose.pose.orientation.z = -0.70682
goal.dock_pose.pose.orientation.w = 0.707388
client.send_goal(goal)
