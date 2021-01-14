#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, Transform
from std_srvs.srv import Empty 

"""
Heavily based on Grzegorr code
https://github.com/Grzegorr/ROSassignment/blob/master/src/ROS/scripts/weedMap.py
"""

class weed_mapper():

    weed_map = [] # Array for weeds

    def __init__(self): #Initialiser
        #Weed arary subscriber 
        self.cloud_sub = rospy.Subscriber("/thorvald_001/points", PointCloud, self.process)
        #Weed map publisher
        self.weedmap_pub = rospy.Publisher("/thorvald_001/weedmap", PointCloud, queue_size=10)
        #Translastion subscriber 
        self.translation_sub = rospy.Subscriber("/thorvald_001/translation", Transform, self.update)
        #Sprayer service
        self.sprayer = rospy.ServiceProxy('/thorvald_001/spray', Empty)

        

    def process(self, data): #process(self, data: PointCloud)
        """
        Process the data to be published on the map.
        """
        
        for point in data.points: #For all points in data
            p = [point.x, point.y,  point.z] # put each point into array
            p = self.tf_transformer(p, self.data.translation, self.data.rotation) #Calculate translation 
            weeds = Point32()
            weeds.x = p[0]
            weeds.y = p[1]
            weeds.z = p[2]
            self.weed_map.append(weeds) # Put points into array
        self.publish_weeds() #Publish the map of weeds
        self.spray(p, self.data.translation) # Spray if possible
        
    
    def spray(self, p, data): # spray(self, p: tuple, data: Vector)
        """
        Spray the weeds if certain circumstances.
        """
        if (data.x + 0.5 > p[0]) and (data.y + 0.1 > p[1]):
            self.sprayer()


    def publish_weeds(self): 
        """
        Publish the weeds ont the map.
        """
        cloud = PointCloud()
        cloud.header.stamp = rospy.Time()
        cloud.header.frame_id = "map"
        cloud.points = self.weed_map
        self.weedmap_pub.publish(cloud) #Publish the weeds 

    
    def update(self, data): # update(self, data: Transform)
        """
        Update the translation.
        """
        self.data = data

    
    def tf_transformer(self,  point,  translation,  rotation): # tf_transformer(self, point: lists[int], translation: Vector, rotation: Quarternion ) -> (x: float, y: float, z:,float, w: float)
        """
        Calculate the transfer frame transform.
        """
        #points conversion with translation
        point[0] = point[0] - translation.x
        point[1] = point[1] - translation.y
        point[2] = point[2] - translation.z
        #create point array
        point = [point[0], point[1], point[2], 0]
        #Create rotation array
        rotation = [rotation.x, rotation.y, rotation.z, rotation.w]
        #Calculate quarternion
        ans = self.quarternion_mult(rotation, point)
        #Inverse rotations
        rot_inv = [-rotation[0], -rotation[1], -rotation[2], rotation[3]]
        #Calculate final point
        return self.quarternion_mult(ans, rot_inv)
    
    def quarternion_mult(self, q1, q2): # quarternion_mult(self, q1: list, ,q2: list ) -> (x: float, y: float, z:,float, w: float)
        """
        Completes quarternion multiplication using formula.
        """
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
        return  x, y, z, w
#Create node
rospy.init_node('Weed_map_publisher')
weed = weed_mapper()
#spin to win
rospy.spin()