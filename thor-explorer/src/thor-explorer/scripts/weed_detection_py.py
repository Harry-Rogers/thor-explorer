#! /usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud
from geometry_msgs.msg import Point32, Transform
import tf


# https://github.com/Grzegorr/ROSassignment
# WAS V HELPFUL FOR MAPPING WEEDS USING PCL
class Weeds():

    def __init__(self):
        self.camsub = rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect", Image, callback=self.cam)
        self.bridge = CvBridge()
        #Change two below and 
        self.transpub = rospy.Publisher("/thorvald_001/capture_time_transform", Transform, queue_size = 10)
        self.pcPub = rospy.Publisher("/thorvald_001/last_frame_points", PointCloud, queue_size=10)
        self.tfListener = tf.TransformListener()


    
    def cam(self, data):
        self.timeStamp = data.header.stamp
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
        params.maxArea = 20000
        params.minArea = 900
        params.minDistBetweenBlobs = 0.01

        detector = cv2.SimpleBlobDetector_create(params)
        # something bout cv2.bitwise_not(filteredImage) instead runs faster app
        dilation = 255 - dilation
        keypoints = detector.detect(dilation)

        keypoints_img = cv2.drawKeypoints(dilation, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        points = cv2.KeyPoint_convert(keypoints)
        #Find the tf transform between the frame of the camera and map
        try:
            (translation,  rotation) = self.tfListener.lookupTransform('thorvald_001/kinect2_rgb_optical_frame',  "map", self.timeStamp)
        except (tf.ExtrapolationException): #Not in sim yet
            return

        transform_to_send = Transform()
        transform_to_send.translation.x = translation[0]
        transform_to_send.translation.y = translation[1]
        transform_to_send.translation.z = translation[2]
        transform_to_send.rotation.x = rotation[0]
        transform_to_send.rotation.y = rotation[1]
        transform_to_send.rotation.z = rotation[2]
        transform_to_send.rotation.w = rotation[3]
        self.transpub.publish(transform_to_send)

        publish_these = []
        for point in points:
            x = int(point[0])
            y = int(point[1])
            publish_these.append([x, y])
        self.pubCloud(publish_these)

        #Show both original and green mask
        #cv2.imshow("Dilated", dilation)
        #cv2.imshow("MedianBlur", median)
        #cv2.imshow("Green Window", mask)
        cv2.imshow("Original", image)
        cv2.imshow("Keypoints", keypoints_img)
        #Wait or error
        cv2.waitKey(1)
    
    def pubCloud(self, points):
        new_point = []
        for point in points:
            x = point[0]
            y = point[1]
            x = (x- 990)* 0.00044
            y = (y - 540)* 0.00044
            cloud = Point32()
            cloud.x = x
            cloud.y = y
            cloud.z = 0.5
            new_point.append(cloud)
        if len(points) > 0:
            pc = PointCloud()
            pc.header.stamp = self.timeStamp
            pc.header.frame_id = "thorvald_001/kinect2_rgb_optical_frame"
            pc.points = new_point
            self.pcPub.publish(pc)
    




#subprocess.call("./spray.sh", shell=True)
#Create node 
rospy.init_node("Weeds")
weed = Weeds()
#spin to win
rospy.spin()