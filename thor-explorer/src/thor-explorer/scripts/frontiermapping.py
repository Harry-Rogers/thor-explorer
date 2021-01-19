#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import random
import move_base_py
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud

"""
Frontier finder code based on Hassan Umari code
https://github.com/hasauino/rrt_exploration/blob/master/scripts/getfrontier.py
"""

class Deployment():
    

    def __init__(self): #Initialiser
        #Map subscirber
        self.map_Sub = rospy.Subscriber("/map", OccupancyGrid, callback=self.mapping)
        #Frontiers publisher
        self.frontier_pub = rospy.Publisher("/frontiers", String, queue_size=10)
        #Iteration value for index flip
        self.index = 0
        #Flag for signalling when the map is done
        self.end = False



    def mapping(self, mapData): #mapping(self, mapData: OccupancyGrid)
        """
        Mapping will find frontiers using the laser scanner.
        Sends frontiers to move base.
        Once end condition is met, goes to origin and another random point.
        """
        rospy.Rate(5)
        #Message to wait
        print("Calculating, this will take a while...")
        #get data from map
        data = mapData.data
        w = mapData.info.width
        h = mapData.info.height
        resolution = mapData.info.resolution
        start_x = mapData.info.origin.position.x
        start_y = mapData.info.origin.position.y

        #create an empty array 
        img = np.zeros((h,w,1), np.uint8)

        
        #for loop tp check all cells in map, very inefficient but will work
        for i in range(0, h):
            for j in range(0, w):
                if data[i*w+j] == 100:
                    img[i,j]=0
                elif data[i*w+j]==0:
                    img[i,j]=255
                elif data[i *w +j] == -1:
                    img[i,j] = 205
            o = cv2.inRange(img, 0,1)

        #Find the edges in the map for areas that are unknown
        edges = cv2.Canny(img, 0, 255)
        contours, hierarchy = cv2.findContours(o, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        del hierarchy #cleanup don't need this
        cv2.drawContours(o, contours, -1, (255,255,255), 5)
        cv2.bitwise_not(o)
        frontier = cv2.bitwise_and(o, edges)
        contours, hierarchy = cv2.findContours(frontier,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        del hierarchy #cleanup don't need this
        cv2.drawContours(frontier, contours, -1, (255,255,255), 2)
        
        all_pts = [] # Create array for points to go into
        if len(contours) > 0: # If there is places to explore
            for i in range(0, len(contours)): #Go through them all
                cnt = contours[i]
                M = cv2.moments(cnt)
                try:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                except ZeroDivisionError: #Some points found will be the origin
                    cx = 0 #set both to 0
                    cy = 0
                xr = cx*resolution + start_x #Convert to metres adding the start point
                yr = cy* resolution + start_y
                if xr and yr == start_x and start_y: #If the values are the same as the start point delete them
                    empty = []
                    empty = [xr,yr]
                    del empty
                else:
                    pt = [np.array([xr,yr])] #If points put them in the array
                    if len(all_pts) >0: #If more than one point to explore
                        all_pts = np.vstack([all_pts, pt]) #Stack them
                    else:
                        all_pts = pt #One point
        self.frontier_pub.publish(str(all_pts)) # Publish the points to Frontiers for debugging

        if all_pts == []: #If there are no more points to explore
            self.end_sequence()
    
        move_base_py.movebase_client(all_pts, self.end, self.index) # Send move base point we want to go to        
        self.index = self.index + 1 # After sending move base add one to the index counter

        if self.index == 3: # If we have explored 3 points we have most if not all of the map explored
            self.end_sequence()
    

    def end_sequence(self):
        """
        Print messages, go to origin then sudo random point using move base.
        """
        print("Map is explored :)")
        print("Going to try and find weeds :)")
        self.end = True
        #Explore some more to find weeds, choose random point first, weeds are likely to be in centre of map so go there after going to the centre of the map
        i = random.uniform(-5,-10)
        #Send to origin point
        originpoint = [0,0]
        move_base_py.movebase_client(originpoint,self.end, self.index)
        move_base_py.movebase_client([i,i],self.end, self.index)
        self.frontier_pub.publish("Stop") #Publish message to stop everything

#Create node 
rospy.init_node("Frontiers")
move = Deployment()
#spin to win
rospy.spin()