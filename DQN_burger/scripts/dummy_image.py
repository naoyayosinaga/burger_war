#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import cv2
import numpy as np



if __name__ == '__main__':
    pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
    rospy.init_node('image_publisher')
    r = rospy.Rate(10) # 10hz
    
    # 640 x 480 CV_8UC3 イメージの生成
    cols = 640
    rows = 480
#    img = np.zeros((rows, cols, 3), dtype=np.uint8)
    img = np.full((rows, cols, 3), 0, dtype=np.uint8)
    img[0:480, 0:640, 1] = 255
#    img[220:260, 200:440, 1] = 200

    bridge = CvBridge()
    imgMsg = bridge.cv2_to_imgmsg(img, "bgr8")


    while not rospy.is_shutdown():
        rospy.loginfo(img)

        pub.publish(imgMsg)

        r.sleep()

