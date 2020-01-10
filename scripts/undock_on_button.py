#!/usr/bin/env python

# Copyright (c) 2015, Fetch Robotics Inc.
# Author: Michael Ferguson

import rospy
import actionlib
from fetch_auto_dock_msgs.msg import UndockAction, UndockGoal

# Create a ROS node
rospy.init_node("undock_the_robot")

# Create an action client
client = actionlib.SimpleActionClient("/undock", UndockAction)
client.wait_for_server()
print("server started!")
# Create and send a goal
goal = UndockGoal()
goal.rotate_in_place = True
client.send_goal(goal)
