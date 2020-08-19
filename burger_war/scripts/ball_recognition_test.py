#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This is rumdom run node.
subscribe No topcs.
Publish 'cmd_vel' topic. 
mainly use for simple sample program

by Takuya Yamaguhi.
'''

import rospy
import random
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

from geometry_msgs.msg import Twist


class RandomBot():
    def __init__(self, use_camera=False, bot_name="NoName"):
        # bot name 
        self.name = bot_name
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

        # camera subscribver
        if use_camera:
            # for convert image topic to opencv obj
            self.img = None
            self.bridge = CvBridge()
            self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)

    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        img = cv2.cvtColor(self.img,cv2.COLOR_BGR2GRAY)
        img = cv2.medianBlur(img,5)
	img = img[0:200, 0:]     	# clip image
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
		print "---------"
		print distance, '[mm]'
		print direction, '[deg]'

        # cv2.imshow("Image window", cimg)
        # cv2.imshow("mask window", cmask)
        cv2.imshow("Image window", cimg)
	cv2.waitKey(1)

    def calcBallDist(self, d_px):
	if d_px <= 0:
	    return 0
	fp = 546.6116		# focal length [px]
	d = 50			# diameter [mm]
	return fp * d / d_px

    def calcBallDirec(self, cx_px):
	fp = 546.6116
	return math.degrees(math.atan2(cx_px, fp))

    def calcTwist(self):
        value = random.randint(1,1000)
        if value < 250:
            x = 0.2
            th = 0
        elif value < 500:
            x = -0.2
            th = 0
        elif value < 750:
            x = 0
            th = 1
        elif value < 1000:
            x = 0
            th = -1
        else:
            x = 0
            th = 0
        twist = Twist()
        # twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        # twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        return twist

    def strategy(self):
        r = rospy.Rate(1) # change speed 1fps

        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0

        while not rospy.is_shutdown():
            twist = self.calcTwist()
            print(twist)
            self.vel_pub.publish(twist)

            r.sleep()


if __name__ == '__main__':
    rospy.init_node('ball_recognition_test')
    bot = RandomBot(False, 'ball_recogtion_bot')
    bot.strategy()

