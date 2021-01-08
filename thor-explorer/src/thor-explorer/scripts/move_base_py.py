#! /usr/bin/env python
import rospy

#Brings in the SimpleActionClient
import actionlib

#Brings in the .action (special messages for actionlib) use by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#to specify in euler thhe required orientation
from tf.transformations import quaternion_from_euler


#Make timer if not at goal after 30s new goal

def movebase_client(all_pts, end):

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    client.wait_for_server()

 
    angle_to_quat = list(quaternion_from_euler(0.0,0.0,0.52)) #random rotation in z direction 

    #creates a new goal using the movebasegoal message
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    x = []
    if end == True:
        goal.target_pose.pose.position.x = all_pts[0]
        goal.target_pose.pose.position.x = all_pts[1]
        
        goal.target_pose.pose.orientation.z = angle_to_quat[2]
        goal.target_pose.pose.orientation.w = angle_to_quat[3]
        client.send_goal(goal)
    
    if end == False:

        i = len(all_pts) % 2
        if i == 0:
            x = len(all_pts) / 2 
            i = x

        try:
            goal.target_pose.pose.position.x = all_pts[i,0]
            print(goal.target_pose.pose.position.x)
            goal.target_pose.pose.position.y = all_pts[i,1]
            print(goal.target_pose.pose.position.y)
        except TypeError:
            pass

    
        goal.target_pose.pose.orientation.z = angle_to_quat[2]
        goal.target_pose.pose.orientation.w = angle_to_quat[3]
        client.send_goal_and_wait(goal)
    
    
        b = client.get_result()
        print(b)
        if b == 3:
            goal.target_pose.pose.position.x = 0
            print(goal.target_pose.pose.position.x)
            goal.target_pose.pose.position.y = 0
            print(goal.target_pose.pose.position.y)
            goal.target_pose.pose.orientation.z = angle_to_quat[2]
            goal.target_pose.pose.orientation.w = angle_to_quat[3]
            client.send_goal_and_wait(goal)
        if b == 4:
            #Failed so we need to save this to arr
            goal_array = []
            goal_array.append(goal)
            #Use these maybe

