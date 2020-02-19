#!/usr/bin/env python
import rospy
from move.msg import commond
import arm_action
import numpy as np

class Arm:

    def __init__(self):
        self.act = arm_action.arm_action()
        self.sub_arm = rospy.Subscriber('Commond_TOP', commond, self.callback)
        self.pub_arm = rospy.Publisher('results_ARM', commond, queue_size=10)

    def callback(self, msg):
        if((msg.node == 3)or(msg.node == 0)):
            send = commond()
            send.topic_id = msg.topic_id
            rospy.loginfo("%d", msg.topic_id)
            if (msg.msg.id == 0):
                self.act.pregrab()
                self.act.grab()
                self.act.postgrab()
                self.act.maintain()
                send.ret = 0
                rospy.loginfo("arm grab")
            elif (msg.msg.id == 1):
                self.act.prerelease()
                self.act.release()
                self.act.pregrab()
                self.act.start_pos()
                send.ret = 0
                rospy.loginfo("arm release")
            else:
                send.ret = 0
            self.pub_arm.publish(send)
        else:
            rospy.loginfo("this topic is not for arm, node=%d", msg.topic_id)

if __name__ == '__main__':
    try:
        arm = Arm()
        rospy.loginfo("running...")
        rospy.spin()
    except rospy.ROSInterruptException: pass
