#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Point
import numpy as np


class Moving:

	def __init__(self):
		self.goal_z = 0.30
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
                self.angular_speed = np.deg2rad(15)
                self.liner_speed = 0.05

	def rotate(self,angle):

		twist = Twist()
		if angle > 0:
			twist.angular.z = self.angular_speed
		if angle < 0:
			twist.angular.z = -self.angular_speed
		print(twist.angular.z)
		t0 = rospy.Time.now().to_sec()
		current_angle = 0
		while(abs(current_angle) < abs(angle)):
			self.pub.publish(twist)
			t1 = rospy.Time.now().to_sec()
			current_angle = self.angular_speed*(t1 - t0)

	def liner_x(self,liner):

		twist = Twist()
		if liner > 0:
			twist.linear.x = self.liner_speed
		if liner < 0:
			twist.linear.x = -self.liner_speed

		distance = liner
		t0 = rospy.Time.now().to_sec()
		current_distance = 0
		while(abs(current_distance) < abs(distance)):
			self.pub.publish(twist)
			t1 = rospy.Time.now().to_sec()
			current_distance = self.liner_speed*(t1 - t0)


	def liner_y(self,liner):

		twist = Twist()
		if liner > 0:
			twist.linear.y = self.liner_speed
		if liner < 0:
			twist.linear.y = -self.liner_speed
		distance = liner
		t0 = rospy.Time.now().to_sec()
		current_distance = 0
		while(current_distance < distance):
			self.pub.publish(twist)
			t1 = rospy.Time.now().to_sec()
			current_distance = self.liner_speed*(t1 - t0)

if __name__ == '__main__':
	rospy.init_node("test")
	try:
		mov= Moving()
        	angle = np.deg2rad(30)
                #print(angle)
        	liner = 0.1
		mov.rotate(angle)
		mov.liner_x(liner)
		#rospy.spin()
	except rospy.ROSInterruptException: pass
