#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class robotnic():

    def __init__(self):
        self.laser_sub = rospy.Subscriber("/thorvald_001/scan", LaserScan, self.dynamic_check)
        self.pub_help = rospy.Publisher("/help", String, queue_size= 10)

    
    def dynamic_check(self, data):
        self.all_laser_data = data.ranges
        close_data = min(self.all_laser_data)
        if close_data < 0.8:
            self.pub_help.publish("Incoming")


rospy.init_node("chek")
move = robotnic()
#spin to win
rospy.spin()