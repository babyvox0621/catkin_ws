#!/usr/bin/env python
import rospy
from move.msg import commond
import move_arm_vo as move_arm
import numpy as np

class Arm:

    def __init__(self):
        self.arm = move_arm.move_arm()
        self.sub_arm = rospy.Subscriber('Commond_TOP', commond, self.callback)
        self.pub_arm = rospy.Publisher('results_ARM', commond, queue_size=1)
        
        self.gripper_open = 95 # 95mm
        self.gripper_close = 45 # 45mm

        # move to start position
        self.pose_upper()
        self.arm.gripper(self.gripper_close)
        self.pose_right_2()
        self.pose_right_1()

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

    def callback(self, msg):
        if((msg.node == 3)or(msg.node == 0)):
            send = commond()
            send.topic_id = msg.topic_id
            rospy.loginfo("%d", msg.topic_id)

            if (msg.msg.id == 0): # grab can

                self.pose_right_2()
                self.pose_upper()
                self.gripper(self.gripper_open)
                self.pose_front()
                self.pose_grip()
                self.arm.grip(self.gripper_close)
                self.pose_front()
                self.pose_upper()
                self.pose_right_2()
                self.pose_right_1()

                send.ret = 0
                rospy.loginfo("grab can")

            elif (msg.msg.id == 1): # release can

                self.pose_right_2()
                self.pose_upper()
                self.pose_front()
                self.pose_release()
                self.arm.grip(self.gripper_open)
                rospy.Rate(0.25).sleep() # sleep 4sec
                self.pose_front()
                self.arm.grip(self.gripper_close)
                self.pose_right_2()
                self.pose_right_1()               
                
                send.ret = 0
                rospy.loginfo("release can")

            else:

                send.ret = 0
                rospy.loginfo("undifined id")

            self.pub_arm.publish(send)
        else:
            rospy.loginfo("this topic is not for arm, node=%d", msg.topic_id)

if __name__ == '__main__':
    try:
        arm = Arm()
        rospy.loginfo("running...")
        rospy.spin()
    except rospy.ROSInterruptException: pass
