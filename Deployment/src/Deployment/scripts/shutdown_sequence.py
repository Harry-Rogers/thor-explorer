import rospy
import os
import subprocess
from std_msgs.msg import String
class shutdown_sequence():

    def __init__(self):
        self.sub = rospy.Subscriber("/frontiers", String, self.stop)
        
    
    def stop(self, data):
        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")
        if data.data == "Stop":
            subprocess.call("./mapmaker.sh", shell = True)
            for node in nodes:
                os.system("rosnode kill "+ node)
        else:
            print("Waiting....")

#Create node 
rospy.init_node("Shutdown")
move = shutdown_sequence()
#spin to win
rospy.spin()

