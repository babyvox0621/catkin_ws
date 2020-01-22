#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Point
import numpy as np


class Cmd:

	def __init__(self):
		self.goal_z = 0.18
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
                self.angular_speed = np.deg2rad(5)
                self.liner_speed = 0.05

	def rotate(self,msg):

		angle = np.arctan2(msg.x, msg.z)
		twist = Twist()
		if angle < 0:
			twist.angular.z = self.angular_speed
		if angle > 0:
			twist.angular.z = -self.angular_speed
		print(twist.angular.z)
		t0 = rospy.Time.now().to_sec()
		current_angle = 0
		while(abs(current_angle) < abs(angle)):
			self.pub.publish(twist)
			t1 = rospy.Time.now().to_sec()
			current_angle = self.angular_speed*(t1 - t0)
	def back_and_forward(self,msg):

		twist = Twist()
		gap = np.sqrt(msg.z**2 + msg.x**2) - self.goal_z
		if gap > 0:
			twist.linear.x = self.liner_speed
		if gap < 0:
			twist.linear.x = -self.liner_speed

		distance = gap
		t0 = rospy.Time.now().to_sec()
		current_distance = 0
		while(abs(current_distance) < abs(distance)):
			self.pub.publish(twist)
			t1 = rospy.Time.now().to_sec()
			current_distance = self.liner_speed*(t1 - t0)

if __name__ == '__main__':
	rospy.init_node("test")
	try:
		cmd = Cmd()
		point = Point()
		point.x = 0.2
		point.y = 0.3
		point.z = 0.5
		cmd.rotate(point)
		cmd.back_and_forward(point)
		rospy.spin()
	except rospy.ROSInterruptException: pass
