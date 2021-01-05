
import rospy
from rosgraph_msgs.msg import Log
from std_msgs.msg import String

class roscheck():

    
    def __init__(self):
        self.ros_sub = rospy.Subscriber("/rosout", Log, self.check)
        self.pub_help = rospy.Publisher("/help", String, queue_size= 10)

    def check(self, data):
        test = data.level
        if test > 4:
            self.pub_help.publish("Go")
        else:
            self.pub_help.publish("Wait")



rospy.init_node("roscheker")
move = roscheck()
#spin to win
rospy.spin()