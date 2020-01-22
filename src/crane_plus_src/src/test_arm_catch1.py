#! /usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
import move_arm_v1 as move_arm

class test_arm:
    def __init__(self):
        ####################
        #move_arm呼出し
        self.arm=move_arm.move_arm()

        self.arm.x=0.0
        self.arm.y=0.2
        self.arm.z=0.1
        self.arm.wrist_arg=0.0
        grip=95
        self.arm.move_xyz()
        self.arm.grip(grip)

        M_PI=3.14

        ##loop
        while(10):
		#catch
                 self.arm.z-=0.03 #0
                 self.arm.y+=0.07 #8
                 #self.arm.x+=0.001 #6
                 grip-=95
                 if grip>100: grip=100
                 self.arm.move_xyz()
                 self.arm.grip(grip) #7

		#catch back
                 self.arm.z+=0.03 #5
                 self.arm.y-=0.07 #2
                 #self.arm.x-=0.001 #4
                 self.arm.move_xyz()
                 grip+=95
                 if grip<0: grip=0
                 self.arm.grip(grip) #1

            #ARM移動
            #self.arm.move_xyz()
            
        exit()

if __name__ == '__main__':
 
    try:
        ts = test_arm()
        rospy.spin()
    except rospy.ROSInterruptException: pass
