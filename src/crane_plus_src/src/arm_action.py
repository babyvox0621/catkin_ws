#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import move_arm_v1 as move_arm

class arm_action:
    def __init__(self):
        ####################
        #move_arm呼出し
        self.arm=move_arm.move_arm()

        self.arm.x=0.0
        self.arm.y=0.15
        self.arm.z=0.10
        self.arm.wrist_arg=0.0
        self.grip=95
        self.arm.move_xyz()
        self.arm.grip(self.grip)

        M_PI=3.14


    def pregrab(self):
		#catch
        self.arm.y = 0.20 #8
        self.arm.z = 0.0 #0
        #self.arm.x+=0.001 #6
        self.grip = 95
        if self.grip>100:
			self.grip = 100
        self.arm.move_xyz()
        self.arm.grip(self.grip) #7

    def grab(self):
		#catch
        self.arm.y = 0.28 #8
        self.arm.z = 0.0 #0
        #self.arm.x+=0.001 #6
        self.grip = 45
        if self.grip>100:
			self.grip = 100
        self.arm.move_xyz()
        self.arm.grip(self.grip) #7

    def maintain(self):
	
        self.arm.y = 0.15 #8
        self.arm.z = 0.10 #0
        #self.arm.x+=0.001 #6
        self.grip = 45
        if self.grip>100:
			self.grip = 100
        self.arm.move_xyz()
        self.arm.grip(self.grip) #7

    def release(self):
		#catch back
        self.arm.y = 0.15 #5
        self.arm.z = 0.10 #2
        #self.arm.x-=0.001 #4
        self.arm.move_xyz()
        self.grip = 95
        if self.grip<0:
			grip=0
        self.arm.grip(self.grip) #1
        print('release')

    def start_pos(self):
		#catch back
        self.arm.x=0.0
        self.arm.y=0.15
        self.arm.z=0.10
        #self.arm.x-=0.001 #4
        self.arm.move_xyz()
        self.grip = 95
        if self.grip<0:
			grip=0
        self.arm.grip(self.grip) #1
        print('release')

if __name__ == '__main__':
    try:
        arm = arm_action()
        #arm.release()
        arm.pregrab()
        arm.grab()
        #arm.maintain()
        #arm.release()
        #arm.start_pos()
        rospy.loginfo("running...")
        #rospy.spin()
    except rospy.ROSInterruptException: pass
