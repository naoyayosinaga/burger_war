#!/usr/bin/env python
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy
from geometry_msgs.msg import Pose2D

# CamBallRecognition (CBR) Subscriber
class CBRSubscriber():
        def __init__(self):
	    rospy.init_node('CBRListener')
	    rospy.Subscriber('cam_ball_relative_pose',Pose2D, self.pose_callback)
	    rospy.Subscriber('cam_ball_recog_img', Image, self.img_callback)

            self.bridge = CvBridge()

	    rospy.spin()

        def pose_callback(self, pose):
	    if pose.x is not None and pose.y is not None:
                print '---------------'
                print 'x:', pose.x, '[mm]'
                print 'y:', pose.y, '[mm]'

        def img_callback(self, msg_img):
            img = self.bridge.imgmsg_to_cv2(msg_img, "bgr8")
	    cv2.imshow("ball recognition", img)
            cv2.waitKey(1)


if __name__ == '__main__':
    cbrs = CBRSubscriber()
