#!/usr/bin/env python
# -*- coding: utf-8 -*-

# level_1_cheese.py
# write by yamaguchi takuya @dashimaki360
## GO and Back only


import rospy
import random

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

class ManualControll():
    def __init__(self):
    	rospy.init_node('enemy')

        # speed [m/s]
        self.speed = 0.4

        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
	self.joy_sub = rospy.Subscriber('joy', Joy, self.JoyCallback)

    	rospy.spin()

    def Joy2Twist(self, joy_msg):
        x = joy_msg.axes[1] * self.speed
        th = joy_msg.axes[0]

        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist

    def JoyCallback(self, msg):
        twist = self.Joy2Twist(msg)
        self.vel_pub.publish(twist)


if __name__ == '__main__':
    bot = ManualControll()
