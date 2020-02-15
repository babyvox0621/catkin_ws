#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Point
import numpy as np
import time


class Adjust:

    def __init__(self):

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.angular_speed = np.deg2rad(5)
        self.linear_speed = 0.05

    def rotate(self, theta):

        twist = Twist()
        # angular.zは回転速度(rad/s)。反時計回りが正。
        if(theta > 0):
            twist.angular.z = self.angular_speed
        if(theta < 0):
            twist.angular.z = -self.angular_speed
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        rate = 30
        R = rospy.Rate(rate)
        ticket = int((theta / twist.angular.z) * rate)
        for i in range(ticket):
            #t1 = rospy.Time.now().to_sec()
            #current_angle = self.angular_speed*(t1 - t0)
            #if(abs(current_angle) > abs(theta)):
            #    break
            self.pub.publish(twist)
            R.sleep()
        twist.angular.z = 0
        self.pub.publish(twist)        
        #time.sleep(1.5)
        rospy.loginfo('finish rotate')

    def back_and_forward(self, distance):

        twist = Twist()
        if(distance > 0):
            twist.linear.x = self.linear_speed
            # linear.xは前後方向の並進速度(m/s)。前方向が正。
        if(distance < 0):
            twist.linear.x = -self.linear_speed
        t0 = rospy.Time.now().to_sec()
        rate = 30
        R = rospy.Rate(rate)
        ticket = int((distance / twist.linear.x) * rate)
        for i in range(ticket):
            #t1 = rospy.Time.now().to_sec()
            #current_angle = self.angular_speed*(t1 - t0)
            #if(abs(current_angle) > abs(theta)):
            #    break
            self.pub.publish(twist)
            R.sleep()
        twist.linear.x = 0
        self.pub.publish(twist)        
        #time.sleep(1.5)
        rospy.loginfo('finish back and forward')

if __name__ == '__main__':
    rospy.init_node("adjust")
    try:
        cmd = Adjust()
        point = Point()
        point.x = 0.2
        point.y = 0.3
        point.z = 0.5
        cmd.rotate(point)
        cmd.back_and_forward(point)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
