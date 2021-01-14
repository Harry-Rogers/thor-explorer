#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import tf
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud
from geometry_msgs.msg import Point32, Transform

"""
Converting to map code based on Grzegorr code
https://github.com/Grzegorr/ROSassignment/blob/master/src/ROS/scripts/weedMap.py
"""

class Weeds():

    def __init__(self):#Initialiser
        #Camera sub
        self.cam_sub = rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect", Image, callback=self.cam)
        #Bridge for opencv
        self.bridge = CvBridge()
        #Translation publisher
        self.translation_pub = rospy.Publisher("/thorvald_001/translation", Transform, queue_size = 10)
        #Weed points publisher
        self.cloud_pub = rospy.Publisher("/thorvald_001/points", PointCloud, queue_size=10)
        #TF transformer
        self.transformer = tf.TransformListener()


    
    def cam(self, data): # cam(self, data: Image)
        """
        Cam will look for weeds from the camera, will publish those points found to be mapped
        """
        self.ros_time = data.header.stamp #Get time stamp
        #Bridge to bgr8
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        median = cv2.medianBlur(image, 25) #Median blur 
        #Hue saturation and intensity
        hsv = cv2.cvtColor(median, cv2.COLOR_BGR2HSV)
        #Green vals to find plants
        lower_green = np.array([45, 50, 50])
        upper_green = np.array([65, 255, 255])
        #create mask
        mask = cv2.inRange(hsv, lower_green, upper_green)
    
        #Use mask
        cv2.bitwise_and(median, median, mask = mask)
        kernel = np.ones((5,5),np.uint8) #Kernel for dilation
        dilation = cv2.dilate(mask, kernel, iterations=3) #Dilation to make blobs bigger
        params = cv2.SimpleBlobDetector_Params() #Create params for blob detector
        
        #params for blobs
        params.filterByArea = True
        params.maxArea = 20000
        params.minArea = 900
        params.minDistBetweenBlobs = 0.01

        detector = cv2.SimpleBlobDetector_create(params) #Detect blobs
        dilation = 255 - dilation #flip
        keypoints = detector.detect(dilation) #find key points

        keypoints_img = cv2.drawKeypoints(dilation, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS) #Draw keypoints on image with red circle

        points = cv2.KeyPoint_convert(keypoints) #Convert keypoints to coords
        #Find the tf transform between the frame of the camera and map
        try:
            (translation,  rotation) = self.transformer.lookupTransform('thorvald_001/kinect2_rgb_optical_frame',  "map", self.ros_time) #Lookup the tf for mapping weeds properly
        except (tf.ExtrapolationException): #Not in sim yet
            return
        # Convert the translation and rotation to be published 
        thor_transform = Transform()
        thor_transform.translation.x = translation[0]
        thor_transform.translation.y = translation[1]
        thor_transform.translation.z = translation[2]
        thor_transform.rotation.x = rotation[0]
        thor_transform.rotation.y = rotation[1]
        thor_transform.rotation.z = rotation[2]
        thor_transform.rotation.w = rotation[3]
        self.translation_pub.publish(thor_transform)

        publish_these = [] # Array for points to be published
        for point in points: #for loop to fill array
            x = int(point[0]) #convert to int and use x,y
            y = int(point[1])
            publish_these.append([x, y]) # append array
        self.pub_cloud(publish_these) #publish the array

        #Show both original and green mask
        #cv2.imshow("Dilated", dilation)
        #cv2.imshow("MedianBlur", median)
        #cv2.imshow("Green Window", mask)
        cv2.imshow("Original", image)
        cv2.imshow("Keypoints", keypoints_img)
        #Wait or error
        cv2.waitKey(1)
    
    def pub_cloud(self, points): #pub_cloud(self, points: list[int])
        """
        Publish the cloud weed data.
        """
        new_point = [] #Create array for points after conversion
        for point in points:
            x = point[0] #unpack points
            y = point[1]
            x = (x- 990)* 0.00044 #Convert for both
            y = (y - 540)* 0.00044
            cloud = Point32() #Send to new array
            cloud.x = x
            cloud.y = y
            cloud.z = 0.5
            new_point.append(cloud) #append the array
        if len(points) > 0: #If there is a point
            pc = PointCloud()
            pc.header.stamp = self.ros_time
            pc.header.frame_id = "thorvald_001/kinect2_rgb_optical_frame"
            pc.points = new_point
            self.cloud_pub.publish(pc) #Publish the point
    
#Create node 
rospy.init_node("Weeds")
weed = Weeds()
#spin to win
rospy.spin()