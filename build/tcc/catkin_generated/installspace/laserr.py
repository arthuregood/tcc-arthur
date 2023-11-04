#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion

def move_to_goal(x, y):
    # Initialize a ROS node
    rospy.init_node('move_to_goal', anonymous=True)

    # Create a SimpleActionClient for move_base
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Create a goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Set the position and orientation
    goal.target_pose.pose.position = Point(x, y, 0.0)
    goal.target_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

    # Send the goal
    client.send_goal(goal)

    # Wait for the result (you can add a timeout here if needed)
    client.wait_for_result()

if __name__ == '__main__':
    # Define the target position (X, Y)
    target_x = 1.0
    target_y = 2.0

    try:
        move_to_goal(target_x, target_y)
        rospy.loginfo("Goal reached successfully!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted")
