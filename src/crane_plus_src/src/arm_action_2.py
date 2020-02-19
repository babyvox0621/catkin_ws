#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import move_arm_vo as move_arm
import numpy as np

class arm_action:
    def __init__(self):
        ####################
        #move_arm呼出し
        self.arm=move_arm.move_arm()
        self.gripper_close = 45
        self.gripper_open = 95

        """
        Tilt
        1:
        2:
        3:
        4:
        5:
        """
        # move to start position
        
        self.pose_upper()
        self.arm.grip(self.gripper_close)
        self.pose_right_2()
        self.pose_right_1()
        

        #self.arm.grip(self.gripper_open)

    def pose_right_1(self): # right position
        self.arm.Tilt[1] = np.pi/2
        self.arm.Tilt[2] = np.pi/2
        self.arm.Tilt[3] = -np.pi/3.5
        self.arm.Tilt[4] = 0
        self.arm.move_tilt()

    def pose_right_2(self): # 
        self.arm.Tilt[1] = np.pi/2
        self.arm.Tilt[2] = np.pi/3
        self.arm.Tilt[3] = -np.pi/3
        self.arm.Tilt[4] = np.pi/2
        self.arm.move_tilt()

    def pose_upper(self): # upper position
        self.arm.Tilt[1] = 0
        self.arm.Tilt[2] = np.pi/3
        self.arm.Tilt[3] = -np.pi/3
        self.arm.Tilt[4] = np.pi/2
        self.arm.move_tilt()

    def pose_front(self): # front position
        self.arm.x = 0
        self.arm.y = 0.05
        self.arm.z = 0.25
        # self.arm.grip(95) # open gripper
        # self.arm.move_xyz()
        self.arm.move_xyz()

    def pose_grip(self):
        self.arm.x = 0
        self.arm.y = 0.00
        self.arm.z = 0.35
        self.arm.move_xyz()

    def pose_release(self):
        self.arm.x = 0
        self.arm.y = 0.2
        self.arm.z = 0.3
        self.arm.move_xyz()

if __name__ == '__main__':
    try:
        arm = arm_action()
        """
        arm.pose_right_2()
        arm.pose_upper()
        arm.arm.grip(arm.gripper_open)
        arm.pose_front()
        arm.pose_grip()
        arm.arm.grip(arm.gripper_close)
        """
        """
        arm.pose_front()
        arm.pose_upper()
        arm.pose_right_2()
        arm.pose_right_1()
        
        rospy.Rate(0.25).sleep()
        arm.pose_right_2()
        arm.pose_upper()
        arm.pose_front()
        arm.pose_release()
        arm.arm.grip(self.gripper_open)
        rospy.Rate(0.25).sleep() # sleep 4sec
        arm.pose_front()
        arm.arm.grip(self.gripper_close)
        arm.pose_
        arm.pose_right_2()
        arm.pose_right_1() 
         """           
                
        #arm.start_pos()
        rospy.loginfo("running...")
        #rospy.spin()
    except rospy.ROSInterruptException: pass
