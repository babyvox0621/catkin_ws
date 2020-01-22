#!/usr/bin/env python
import rospy
import subprocess
import time
from move.msg import commond

class Dummy:

    def __init__(self):

        self.pub_video = rospy.Publisher('Commond_TOP', commond, queue_size=10)
    
    def id1(self):
        send = commond()
        send.node = 4
        send.msg.id = 0
        self.pub_video.publish(send)
        rospy.loginfo("publish id1")

    def id2(self):
        send = commond()
        send.node = 4
        send.msg.id = 1
        self.pub_video.publish(send)
        rospy.loginfo("publish id2")

if __name__ == '__main__':
    rospy.init_node('dummy_commond')
    try:
        dummy = Dummy()
        #rospy.loginfo("running...")
        dummy.id1()
        time.sleep(5)
        dummy.id2()
        time.sleep(5)
        dummy.id1()
        time.sleep(5)
        dummy.id2()
        #rospy.spin()
    except rospy.ROSInterruptException: pass
