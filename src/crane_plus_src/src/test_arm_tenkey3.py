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
        grip=0
        self.arm.move_xyz()
        self.arm.grip(grip)

        M_PI=3.14

        ##loop
        while(1):
            key = raw_input('key=> ')

            if key=="q":
                break
            elif key=="8": #forward
                self.arm.y+=0.01
            elif key=="2": #back
                self.arm.y-=0.01
            elif key=="6": #right
                self.arm.x+=0.01
            elif key=="4": #left
                self.arm.x-=0.01
            elif key=="5": #up
                self.arm.z+=0.01
            elif key=="0": #down
                self.arm.z-=0.01
            elif key=="9": #wrist up
                self.arm.wrist_arg-=M_PI/180*5
            elif key=="3": #wrist down
                self.arm.wrist_arg+=M_PI/180*5
            elif key=="7": #tilt5 close
                grip+=10
                if grip>100: grip=100
                self.arm.grip(grip)
            elif key=="1": #tilt5 open
                grip-=10
                if grip<0: grip=0
                self.arm.grip(grip)

            #ARM移動
            self.arm.move_xyz()
            
        exit()

if __name__ == '__main__':
 
    try:
        ts = test_arm()
        rospy.spin()
    except rospy.ROSInterruptException: pass
