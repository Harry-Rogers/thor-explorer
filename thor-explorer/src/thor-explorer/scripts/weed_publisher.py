#! /usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, Transform
from std_srvs.srv import Empty 
# https://github.com/Grzegorr/ROSassignment
# WAS V HELPFUL FOR MAPPING WEEDS USING PCL
class weedmap():

    weedMap = []

    def __init__(self):
        self.pcSub = rospy.Subscriber("/thorvald_001/last_frame_points", PointCloud, self.process)
        self.weedmapPub = rospy.Publisher("/thorvald_001/weedmap", PointCloud, queue_size=10)
        self.transSub = rospy.Subscriber("/thorvald_001/capture_time_transform", Transform, self.updateTransformer)
        self.sprayer = rospy.ServiceProxy('/thorvald_001/spray', Empty)

        

    def process(self, data):
        for point in data.points:
            p = [point.x, point.y,  point.z]
            p = self.myTF(p, self.data.translation, self.data.rotation)
            weeds = Point32()
            weeds.x = p[0]
            weeds.y = p[1]
            weeds.z = p[2]
            self.weedMap.append(weeds)
        self.publishMap()
        self.spray(p, self.data.translation)
        
    
    def spray(self, p, data):
        if (data.x + 0.5 > p[0]) and (data.y + 0.1 > p[1]):
            self.sprayer()


    def publishMap(self):
        pc = PointCloud()
        pc.header.stamp = rospy.Time()
        pc.header.frame_id = "map"
        pc.points = self.weedMap
        self.weedmapPub.publish(pc)

    
    def updateTransformer(self, data):
        self.data = data

    
    def myTF(self,  point,  trans,  rot):
        #translation
        point[0] = point[0] - trans.x
        point[1] = point[1] - trans.y
        point[2] = point[2] - trans.z
        point = [point[0], point[1], point[2], 0]
        #rotation
        rot = [rot.x, rot.y, rot.z, rot.w]
        ans = self.quaternionMultiplication(rot, point)
        rot_inv = [-rot[0], -rot[1], -rot[2], rot[3] ]
        return self.quaternionMultiplication(ans, rot_inv)
    
    def quaternionMultiplication(self, q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
        return  x, y, z, w

rospy.init_node('weed_map_node')
WM = weedmap()
rospy.spin()