#! /usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class helper():

    def __init__(self):
        self.go_time = rospy.Subscriber("/help", String, self.help)
        self.pubGo = rospy.Publisher("/thorvald_001/twist_mux/cmd_vel", Twist, queue_size=10)
    
    def help(self, data):
        Go = Twist()
        if data.data == 'Go':
            Go.angular.z = 0.5
            self.pubGo.publish(Go)
        elif data.data == "Incoming":
            Go.linear.x = -1
            self.pubGo.publish(Go)
        else:
            print("Waiting")
            
            


rospy.init_node("roshelper")
move = helper()
#spin to win
rospy.spin()