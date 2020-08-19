#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import math
from geometry_msgs.msg import Pose2D


class CamBallRecognition():
    def __init__(self):
        rospy.init_node('cam_ball_recognition')
        self.pub_pose = rospy.Publisher('cam_ball_relative_pose', Pose2D, queue_size=10)
        self.pub_img = rospy.Publisher('cam_ball_recog_img', Image, queue_size=10)
        self.r = rospy.Rate(10) # [hz]
	self.pose = Pose2D() 
	self.recog_img = Image()

        # for convert image topic to opencv obj
        self.img = None
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)
        
    def talker(self):
        while not rospy.is_shutdown():
            # rospy.loginfo(str)
            self.pub_pose.publish(self.pose)
            self.pub_img.publish(self.recog_img)
	    # pose = Pose2D()
	    # pose.x = 20
            # self.pub.publish(pose)
            self.r.sleep()

    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        img = cv2.cvtColor(self.img,cv2.COLOR_BGR2GRAY)
        img = cv2.medianBlur(img,5)
        img = img[0:200, 0:]            # clip image
        cimg = self.img

        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, (0, 20, 0), (5, 255,255))
        mask2 = cv2.inRange(hsv, (95, 20, 0), (100, 255,255))
        mask = cv2.bitwise_or(mask1, mask2)
        mask = cv2.blur(mask,(3,3))
        rows = mask.shape[0]

        circles = cv2.HoughCircles(mask,cv2.HOUGH_GRADIENT,1,40,
                            param1=30,param2=15,minRadius=5,maxRadius=100)

        cmask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
	distance = None
	direction = None

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
                # draw the outer circle
                cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
                cv2.circle(cmask,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
                cv2.circle(cmask,(i[0],i[1]),2,(0,0,255),3)

                distance = self.calcBallDist( i[2]*2 )
                direction = self.calcBallDirec(i[0]-self.img.shape[1]/2)
                # print "---------"
                # print distance, '[mm]'
                # print direction, '[rad]'

        # cv2.imshow("Image window", cimg)
        # cv2.imshow("mask window", cmask)
        # cv2.imshow("Image window", cimg)
        # cv2.waitKey(1)
        bridge = CvBridge()
        msg = bridge.cv2_to_imgmsg(cimg, encoding="bgr8")
	self.recog_img = msg
	
	if distance is None or direction is None:
	    self.pose.x = 0
	    self.pose.y = 0
	    self.pose.theta = 0
	else:
	    self.pose.x = distance * math.cos(direction)
	    self.pose.y = -distance * math.sin(direction)
	    self.pose.theta = 0
	    # print "---------"
            # print 'pose x:', self.pose.x, '[mm]'
            # print 'pose y:', self.pose.y, '[mm]'


    def calcBallDist(self, d_px):
        if d_px <= 0:
            return 0
        fp = 546.6116           # focal length [px]
        d = 50                  # diameter [mm]
        return fp * d / d_px

    def calcBallDirec(self, cx_px):
        fp = 546.6116
        return math.atan2(cx_px, fp)

class CBRSubscriber():
	def __init__(self):
            self.subscriber = rospy.Subscriber('cam_ball_relative_pose', Pose2D, self.callback)
            self.message = Pose2D()

	def callback(self, pose):
	    print '---------------'
            print 'x:', pose.x, '[mm]'
            print 'y:', pose.y, '[mm]'


if __name__ == '__main__':
    try:
        cbr = CamBallRecognition()
	cbr.talker()
        cbrs = CBRSubscriber()

    except rospy.ROSInterruptException: pass
