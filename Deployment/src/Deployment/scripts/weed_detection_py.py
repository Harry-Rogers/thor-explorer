#! /usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class Weeds():

    def __init__(self):
        self.camsub = rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect", Image, callback=self.cam)
        self.bridge = CvBridge()
    
    def cam(self, data):
        #Bridge to bgr8
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        median = cv2.medianBlur(image, 25) 
        #Hue saturation and intensity
        hsv = cv2.cvtColor(median, cv2.COLOR_BGR2HSV)
        #Green vals to find plants
        lower_green = np.array([45, 50, 50])
        upper_green = np.array([65, 255, 255])
        # 16,30,15
        #create mask
        mask = cv2.inRange(hsv, lower_green, upper_green)
    
        #Use mask
        cv2.bitwise_and(median, median, mask = mask)
        kernel = np.ones((5,5),np.uint8)
        dilation = cv2.dilate(mask, kernel, iterations=3)
        params = cv2.SimpleBlobDetector_Params()


        params.filterByArea = True
        params.filterByInertia = True
        params.maxArea = 20000
        params.minArea = 900
        params.minDistBetweenBlobs = 0.01

        detector = cv2.SimpleBlobDetector_create(params)
        # something bout cv2.bitwise_not(filteredImage) instead runs faster app
        dilation = 255 - dilation
        keypoints = detector.detect(dilation)

        keypoints_img = cv2.drawKeypoints(dilation, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        #Show both original and green mask
        cv2.imshow("Dilated", dilation)
        cv2.imshow("MedianBlur", median)
        cv2.imshow("Green Window", mask)
        cv2.imshow("Original", image)
        cv2.imshow("Keypoints", keypoints_img)
        #Wait or error
        cv2.waitKey(1)


#subprocess.call("./spray.sh", shell=True)
#Create node 
rospy.init_node("Weeds")
weed = Weeds()
#spin to win
rospy.spin()