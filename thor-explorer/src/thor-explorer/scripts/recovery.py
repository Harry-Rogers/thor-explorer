
import rospy
from rosgraph_msgs.msg import Log
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import time
#use time checker to see if lines have vhanged if not do this

class roscheck():

    
    def __init__(self):
        self.ros_sub = rospy.Subscriber("/rosout", Log, self.check)
        self.laser_sub = rospy.Subscriber("/thorvald_001/scan", LaserScan, self.help)
        self.pub_help = rospy.Publisher("/thorvald_001/twist_mux/cmd_vel", Twist, queue_size= 10)
        self.flag_close = False
        self.flag_long = False
        self.flag_spin = False
    
        #self.lines = 0

    def check(self, data):
        #time.time.no
        Go = Twist()
        error = data.level
        msg = data.msg
        rotater = True

        if msg == "Rotate recovery can't rotate in place because there is a potential collision. Cost: -1.00":
            while(rotater):
                if self.left > self.right:
                    Go.angular.z = np.pi
                    self.pub_help.publish(Go)
                if self.right > self.left:
                    Go.angular.z = -np.pi
                    self.pub_help.publish(Go)
                if self.middle and self.left and self.right > 1:
                    rotater = False
                

            Go.angular.z = np.pi
            self.pub_help.publish(Go)
            print("Rotating")

        if msg == "Aborting because a valid plan could not be found. Even after executing all recovery behaviors" and self.flag_spin == True and self.flag_long == True:
            Go.linear.x = 1
            Go.angular.z = np.pi
            self.pub_help.publish(Go)
            time.sleep(1)
            Go.angular.z = 0
            self.pub_help.publish(Go)
            print("Deadly error")
            self.flag_spin = False
            self.flag_long = False
        
        if error > 4 and self.flag_close == True:
            Go.linear.x = -0.5
            self.pub_help.publish(Go)
            print("Reversing out of error")
            self.flag_close = False
        if error > 4 and self.flag_long == True:
            Go.linear.x = 1
            self.pub_help.publish(Go)
            print("Going forward no error")
            self.flag_long = False


        self.flag_spin = False
        self.flag_long = False
        self.flag_close = False


        

        

    def help(self, data):
        self.alldata = data.ranges
        self.right = data.ranges[0]
        self.left  = data.ranges[len(self.alldata)-1]
        self.middle = data.ranges[len(self.alldata)/2]

        #test(left, middle,right)
    
        if self.middle < 1.5:
            self.flag_close = True
        if self.middle and self.left and self.right> 3:
            self.flag_long = True
        if self.middle and self.left and self.right > 0.8:
            self.flag_spin = True



rospy.init_node("Roscheck")
move = roscheck()
#spin to win
rospy.spin()