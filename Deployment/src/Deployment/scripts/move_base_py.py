#! /usr/bin/env python
import rospy

#Brings in the SimpleActionClient
import actionlib

#Brings in the .action (special messages for actionlib) use by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#to specify in euler thhe required orientation
from tf.transformations import quaternion_from_euler

import random

#Make timer if not at goal after 30s new goal

def movebase_client(all_pts):

    #Creates an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    #waits until the action server has started up and started listning for goals
    client.wait_for_server()

    
    angle_to_quat = list(quaternion_from_euler(0.0,0.0,0.52)) #random rotation in z direction (by 30 degrees)

    #creates a new goal using the movebasegoal message
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # 5.0 5.0 - Original
    i = random.randint(0,(len(all_pts)-1))
    try:
        goal.target_pose.pose.position.x = all_pts[i,0]
        goal.target_pose.pose.position.y = all_pts[i,1]
        print(all_pts[i, 0])
        print(all_pts[i, 1])
    except (TypeError, IndexError):
        pass
    
    goal.target_pose.pose.orientation.z = angle_to_quat[2]
    goal.target_pose.pose.orientation.w = angle_to_quat[3]
    #send the goal to the action server
    client.send_goal(goal)
    
    
    #result of executing the action
    print(client.get_result)