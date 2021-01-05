#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Frontiers detector, finds areas that are unkown and lead to more exploration of the map.
When the map is complete and there are no more unkown cells in occupancy grid stop
"""
'''
TODO
Stop when map is complete, seems to loop even tho map is done
Integrate weed detection
Find way to find weeds on a map ie not occ grid as too similar with dynamic map
Look over other code to find PCL stuff or whatever they use
Split up files??

'''


import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
import cv2
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
import matplotlib.pyplot as plt
import random
import move_base_py



class Deployment():
    
    def __init__(self):
        self.mapSub = rospy.Subscriber("/map", OccupancyGrid, callback=self.mapping)
        self.frontierpub = rospy.Publisher("/frontiers", String, queue_size=10)

    def mapping(self, mapData):
        print("Calculating, this will take a while...")
        
        data=mapData.data
        w=mapData.info.width
        h=mapData.info.height
        resolution = mapData.info.resolution
        Xstartx= mapData.info.origin.position.x
        Xstarty = mapData.info.origin.position.y


        img = np.zeros((h,w,1), np.uint8)
        
        for i in range(0, h):
            for j in range(0, w):
                if data[i*w+j] == 100:
                    img[i,j]=0
                elif data[i*w+j]==0:
                    img[i,j]=255
                elif data[i *w +j] == -1:
                    img[i,j] = 205
            o = cv2.inRange(img, 0,1)
        edges =cv2.Canny(img, 0, 255)
        
        #plt.imshow(edges)
        #plt.show()
        #plt.show(img)
        contours, hierarchy = cv2.findContours(o, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        del hierarchy
        cv2.drawContours(o, contours, -1, (255,255,255), 5)
        o = cv2.bitwise_not(o)
        #cv2.imshow("o",o)
        #plt.imshow(o)
        #plt.show()
        frontier = cv2.bitwise_and(o, edges)
        #cv2.imshow("frontier",frontier)
        #plt.imshow(frontier)
        #plt.show()
        contours, hierarchy = cv2.findContours(frontier,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        del hierarchy
        cv2.drawContours(frontier, contours, -1, (255,255,255), 2)
        plt.imshow(edges)
        plt.show()
        plt.imshow(o)
        plt.show()
        plt.imshow(frontier)
        plt.show()

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
        print(all_pts)

        move_base_py.movebase_client(all_pts)

        if all_pts == []:
            print("Map is explored :)")
            self.frontierpub.publish("Stop")
            rospy.is_shutdown()



        
  

#subprocess.call("./spray.sh", shell=True)
#Create node 
rospy.init_node("Frontiers")
move = Deployment()
#spin to win
rospy.spin()