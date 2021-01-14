#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler



def movebase_client(all_pts, end, index): # movebase_client(all_pts: ndarray, end: Bool, index: int)
    """
    Move base client, takes points and goes to those points using move base
    Config is set for A*
    """

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) #Make actionclient 

    client.wait_for_server() # Wait for server to be up
 
    angle_to_quat = list(quaternion_from_euler(0.0,0.0,0.52)) #Rotation so it will be facing north west 

    #creates a new goal using the movebasegoal message
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map" 
    goal.target_pose.header.stamp = rospy.Time.now()
    
    
    if end == True: # if end is true should only be origin and point nearby
        goal.target_pose.pose.position.x = all_pts[0]
        goal.target_pose.pose.position.x = all_pts[1]
        
        goal.target_pose.pose.orientation.z = angle_to_quat[2]
        goal.target_pose.pose.orientation.w = angle_to_quat[3]
        client.send_goal_and_wait(goal)
        
    
    if end == False: # if end is false 
        i = len(all_pts) - 1 #interested point is equal to len of points
        if i < 0: #if no points 
            i = 0 #index is 0
        if index % 2 != 0: # if index is even 
            i = 0 # index is 0
        try:
            goal.target_pose.pose.position.x = (all_pts[i,0]) - 1 #Need to get near those points not to them exactly
            goal.target_pose.pose.position.y = (all_pts[i,1]) - 1
        except TypeError, IndexError:
            i = len(all_pts) % 2 
            if i == 0: # if i is even
                x = len(all_pts) / 2 
                i = x # use middle value
            goal.target_pose.pose.position.x = (all_pts[i,0]) - 1
            goal.target_pose.pose.position.y = (all_pts[i,1]) - 1
            pass
        # Send angles and wait for the goal to be complete
        goal.target_pose.pose.orientation.z = angle_to_quat[2]
        goal.target_pose.pose.orientation.w = angle_to_quat[3]
        client.send_goal_and_wait(goal)