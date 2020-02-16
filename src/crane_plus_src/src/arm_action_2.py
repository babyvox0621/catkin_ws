#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import move_arm_v1 as move_arm
import numpy as np

class arm_action:
    def __init__(self):
        ####################
        #move_arm呼出し
        self.arm=move_arm.move_arm()
        for i in range(6):
            self.arm.Tilt[i] = 0
        """
        Tilt
        1:
        2:
        3:
        4:
        5:
        """
        l23 = 0.083
        l34 = 0.0935


        # catch pose
        self.arm.Step = 2
        self.arm.Tilt[2] = np.pi/4
        self.arm.Tilt[3] = np.pi/3
        self.arm.Tilt[4] = self.arm.Tilt[2] - self.arm.Tilt[3] - np.pi/18
        Y = l23*np.cos(self.arm.Tilt[2]) + l34*np.cos(self.arm.Tilt[3])
        Z = l23*np.sin(self.arm.Tilt[2]) - l34*np.sin(self.arm.Tilt[3])

        
        # pre catch
        self.arm.Step = 2
        self.arm.Tilt[2] = np.pi/2.5
        self.arm.Tilt[3] = np.arcsin(-(Z - l23*np.sin(self.arm.Tilt[2]))/l34)
        print(self.arm.Tilt[3])
        #self.arm.Tilt[2] = np.pi/4
        self.arm.Tilt[4] = self.arm.Tilt[2] - self.arm.Tilt[3] - np.pi/18
        

        """
        # post catch
        self.arm.Step = 1
        self.arm.Tilt[3] = np.pi/18
        self.arm.Tilt[2] = np.arccos((Y - l34*np.cos(self.arm.Tilt[3]))/l23)
        self.arm.Tilt[2] = np.pi/4
        self.arm.Tilt[4] = self.arm.Tilt[2] - self.arm.Tilt[3] - np.pi/18
        """

        """      
        #left position
        self.arm.Tilt[1] = np.pi/2
        self.arm.Tilt[2] = -np.pi/2
        self.arm.Tilt[3] = np.pi/3.5
        self.arm.Tilt[4] = 0
        """
        self.arm.move_tilt()

    # def catch_01(self):



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
        #arm.pregrab()
        #arm.grab()
        #arm.maintain()
        #arm.release()
        #arm.start_pos()
        rospy.loginfo("running...")
        #rospy.spin()
    except rospy.ROSInterruptException: pass
