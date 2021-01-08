import rospy
import os
import subprocess
import actionlib
from move_base_msgs.msg import MoveBaseAction
from std_msgs.msg import String

class shutdown_sequence():

    def __init__(self):
        self.sub = rospy.Subscriber("/frontiers", String, self.stop)
        
    
    def stop(self, data):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")
        if data.data == "Stop":
            subprocess.call("./mapmaker.sh", shell = True)
            for node in nodes:
                if node != "Shutdown":
                    client.cancel_all_goals()
                    os.system("rosnode kill "+ node)
                elif len(node) == 1:
                    os.system("rosnode kill "+ node)

        else:
            print("Waiting....")

#Create node 
rospy.init_node("Shutdown")
move = shutdown_sequence()
#spin to win
rospy.spin()

