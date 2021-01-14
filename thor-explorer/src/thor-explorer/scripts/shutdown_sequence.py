#!/usr/bin/env python3
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
        nodes = os.popen("rosnode list").readlines() # read all nodes
        for i in range(len(nodes)): # for i in all nodes
            nodes[i] = nodes[i].replace("\n","") # Create array of nodes
        if data.data == "Stop": # If mapping is done
            subprocess.call("./mapmaker.sh", shell = True) #Run mapmaker that will save the map
            for node in nodes:
                if node != "Shutdown": #if node is not this node kill all others
                    os.system("rosnode kill "+ node)
                elif len(node) == 1: #if there is only one node left kill ie this node
                    os.system("rosnode kill "+ node)

        else:
            print("Waiting....") #Wait

#Create node 
rospy.init_node("Shutdown")
shutdown = shutdown_sequence()
#spin to win
rospy.spin()