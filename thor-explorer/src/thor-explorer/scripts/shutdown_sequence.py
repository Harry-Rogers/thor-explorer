#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
import subprocess
from std_msgs.msg import String

class shutdown_sequence():

    def __init__(self):#Initialiser
        # Frontiers subscriber
        self.sub = rospy.Subscriber("/frontiers", String, self.stop)
        
    
    def stop(self, data): #stop(self, data: String)
        """
        Stops all nodes created once the mapping is complete
        """
        #nodes = os.popen("rosnode list").readlines() # read all nodes
        #for i in range(len(nodes)): # for i in all nodes
        #    nodes[i] = nodes[i].replace("\n","") # Create array of nodes
        if data.data == "Stop": # If mapping is done
            os.system("rosrun thor-explorer mapmaker.sh")
            os.system("rosnode kill frontiermapping")
            os.system("rosnode kill weed_detection_py")
            os.system("rosnode kill weed_publisher")
            os.system("rosnode kill shutdown_sequence")
            #for node in nodes:
             #   print(node)
             #   if node != "rviz": #if node is not this node or rviz kill all others
             #       os.system("rosnode kill "+ node)
        else:
            print("Waiting....") #Wait

#Create node 
rospy.init_node("Shutdown")
shutdown = shutdown_sequence()
#spin to win
rospy.spin()