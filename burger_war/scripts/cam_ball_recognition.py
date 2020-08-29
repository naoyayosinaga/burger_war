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
		self.pub_d_th = rospy.Publisher('cam_ball_d_th', Pose2D, queue_size=10)
		self.pub_img = rospy.Publisher('cam_ball_recog_img', Image, queue_size=10)
		self.pub_img_b = rospy.Publisher('cam_ball_recog_img_b', Image, queue_size=10)

		self.r = rospy.Rate(10) # [hz]
		self.pose = Pose2D() 
		self.d_th = Pose2D() 
		self.recog_img = Image()
		self.recog_img_b = Image()

		# for convert image topic to opencv obj
		self.img = None
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)
		self.fp = 694.13366           	# focal length [px]
		self.cam_ball_height = 200		# [mm]
		self.ball_d = 65                # diameter [mm]
		self.ma_distance = MovingAverage()
		self.ma_direction = MovingAverage()
	
	def talker(self):
		while not rospy.is_shutdown():
			if self.pose is not None and self.d_th is not None:
				self.pub_pose.publish(self.pose)
				self.pub_d_th.publish(self.d_th)
			self.pub_img.publish(self.recog_img)
			self.pub_img_b.publish(self.recog_img_b)
			self.r.sleep()

	def imageCallback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			rospy.logerr(e)

		img_b = self.BinaryProcessing(self.img)
		cimg_b= cv2.cvtColor(img_b, cv2.COLOR_GRAY2BGR)
		msg = self.bridge.cv2_to_imgmsg(cimg_b, encoding="bgr8")
		self.recog_img_b = msg

		distance, direction, img = self.HoughBallRecog(img_b, self.img)

		msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
		self.recog_img = msg
		
		distance = self.ma_distance.add(distance)
		direction = self.ma_direction.add(direction)

		self.d_th = self.SetDandTh(distance, direction)
		self.pose = self.SetPose(distance, direction)

	def BinaryProcessing(self, color_img):
		result_img = np.zeros(color_img.shape[0:2], np.uint8)
		color_img = color_img[0:300, 0:]            # clip image
		
		# hue threshold
		hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
		masked_img  = cv2.inRange(hsv, (-10, 100, 100), (10, 255,255))
		
		# remove noise by morphology trans.
		kernel_morph = np.ones((5,5), np.uint8)
		masked_img = cv2.morphologyEx(masked_img, cv2.MORPH_CLOSE, kernel_morph)
		masked_img = cv2.blur(masked_img,(5,5))
		result_img[0:300, 0:] = masked_img
		
		return result_img

	def HoughBallRecog(self, img_b, img_c): # img_b: binary, img_c: color
		distance = None
		direction = None
		
		circles = cv2.HoughCircles(img_b,cv2.HOUGH_GRADIENT,1,40,
		                    param1=30,param2=15,minRadius=5,maxRadius=100)
		
		if circles is not None:
		    circles = np.uint16(np.around(circles))
		    for i in circles[0,:]:
		        cv2.circle(img_c,(i[0],i[1]),i[2],(0,255,0),2) 		# outline
		        cv2.circle(img_c,(i[0],i[1]),2,(0,0,255),3) 		# center
		
		        distance = self.CalcBallDist( i[2]*2 )
		        direction = self.CalcBallDirec(i[0]-img_b.shape[1]/2)
		
		return distance, direction, img_c
	
	def SetDandTh(self, d, th):
		if d is None or th is None:
			return None
	
		d_th = Pose2D()
		d_th.x = d
		d_th.y = 0
		d_th.theta = math.degrees(th) # rad2deg
		return d_th

	def SetPose(self, d, th):
		if d is None or th is None:
			return None

		x = 0
		y = 0
		th_p = 0
		
		if d > 1:
		    h = self.cam_ball_height
		    x = math.sqrt(d*d - h*h) * math.cos(th)
		    y = -math.sqrt(d*d - h*h) * math.sin(th)
		
		pose = Pose2D()
		pose.x = x
		pose.y = y
		pose.theta = th_p
		
		return pose

	def CalcBallDist(self, d_px):
		if d_px <= 0:
		    return 0
		return self.fp * self.ball_d / d_px
	
	def CalcBallDirec(self, cx_px):
		return math.atan2(cx_px, self.fp)


class MovingAverage():
    def __init__(self):
		self.size = 5
		self.is_full = False
		self.is_preNone = False
		self.values = [0] * self.size
		self.index = 0
		self.result = 0

    def add(self, v):
		if v is None:
			if self.is_preNone: return None

			self.is_preNone = True
			return self.result

		self.values[self.index] = v
		self.is_preNone = False

		self.index += 1
		if self.index >= self.size:
		     self.index = 0
		     self.is_full = True

		if self.is_full:
		    return self.average()
		else:
			return 0

    def average(self):
  		max_v = max(self.values)
  		min_v = min(self.values)
		sum_v = sum(self.values)
		self.result = (sum_v - max_v - min_v) / (self.size - 2)

		return self.result


if __name__ == '__main__':
    try:
        cbr = CamBallRecognition()
	cbr.talker()

    except rospy.ROSInterruptException: pass
