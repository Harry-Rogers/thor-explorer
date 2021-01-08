#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
import cv2
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point32
from std_msgs.msg import String
import matplotlib.pyplot as plt
from sensor_msgs.msg import PointCloud
import random
import move_base_py

"""
https://github.com/hasauino/rrt_exploration/blob/master/scripts/getfrontier.py
Source code for getting frontiers
Loops through all cells in the map
Checks for areas that are unknown from the laser scanner
Sends x,y coords of points to move base file
If all points are explored send signal to shutdown
Takes a long time due to map being enormous
"""

class Deployment():
    

    def __init__(self):
        self.mapSub = rospy.Subscriber("/map", OccupancyGrid, callback=self.mapping)
        self.frontierpub = rospy.Publisher("/frontiers", String, queue_size=10)
        self.weed_sub = rospy.Subscriber("/thorvald_001/weedmap", PointCloud, callback = self.weedfinder)



    def mapping(self, mapData):
        rospy.Rate(5)
        #Message to wait 
        print("Calculating, this will take a while...")
        #get data from map
        data=mapData.data
        w=mapData.info.width
        h=mapData.info.height
        resolution = mapData.info.resolution
        Xstartx= mapData.info.origin.position.x
        Xstarty = mapData.info.origin.position.y

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
        edges =cv2.Canny(img, 0, 255)        
        contours, hierarchy = cv2.findContours(o, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        del hierarchy #cleanup don't need this
        x =cv2.drawContours(o, contours, -1, (255,255,255), 5)
        o = cv2.bitwise_not(o)
        frontier = cv2.bitwise_and(o, edges)
        contours, hierarchy = cv2.findContours(frontier,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        del hierarchy #cleanup don't need this
        a = cv2.drawContours(frontier, contours, -1, (255,255,255), 2)

        all_pts = []
        if len(contours) > 0:
            for i in range(0, len(contours)):
                cnt = contours[i]
                M = cv2.moments(cnt)
                try:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                except ZeroDivisionError: 
                    cx = 0
                    cy = 0
                xr = cx*resolution + Xstartx
                yr = cy* resolution + Xstarty
                if xr and yr == Xstartx and Xstarty:
                    empty = []
                    empty = [xr,yr]
                    del empty
                else:
                    pt = [np.array([xr,yr])]
                    if len(all_pts) >0:
                        all_pts = np.vstack([all_pts, pt])
                    else:
                        all_pts = pt
        self.frontierpub.publish(str(all_pts))
        if all_pts == []:
            print("Map is explored :)")
            print("Going to try and find weeds :)")
            end = True
            #Explore some more to find weeds, choose previous point thats far away
            #do 0 first then 3 random
            i = random.uniform(-5,-10)
            j = random.uniform(5,10)

            #Send to origin point
            # Change other points to if there are previous weeds mapped, use points near those +-5
            originpoint = [0,0]
            print([i,i])
            print([j,j])
            move_base_py.movebase_client(originpoint,end)
            move_base_py.movebase_client([i,i],end)
            self.frontierpub.publish("Stop")

        print(all_pts)
        end = False
        move_base_py.movebase_client(all_pts, end)

    def weedfinder(self, data):
        weeds = 0
        if data.points:
            weeds = len(data.points)

        try:
            weeds/3
        except ZeroDivisionError:
            move_base_py.movebase_client([0,0], end= True)




        
  

#Create node 
rospy.init_node("Frontiers")
move = Deployment()
#spin to win
rospy.spin()