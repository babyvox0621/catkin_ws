#!/usr/bin/env python
import rospy
import subprocess
import time
from move.msg import commond

class Voice:

    def __init__(self):

        self.sub_video = rospy.Subscriber('Commond_TOP', commond, self.callback)
        self.pub_video = rospy.Publisher('results_VOICE', commond, queue_size=1)
        self.filename = 'cigarette_cm.mp4'
        rospy.loginfo('init finished')

    def callback(self, msg):
        if((msg.node == 4)or(msg.node == 0)):
            send = commond()
            send.topic_id = msg.topic_id
            rospy.loginfo("%d", msg.topic_id)
            if(msg.msg.id == 0):
                rospy.loginfo("waiting...")
                time.sleep(20)
                send.ret = 0

            elif(msg.msg.id == 1):
                print(self.filename)
                totem = subprocess.Popen(['totem', '/home/ubuntu/catkin_ws/src/voice/scripts/cigarette_cm.mp4'])
                time.sleep(40)
                totem.kill()
                send.ret = 0
            rospy.loginfo("finish action")
            self.pub_video.publish(send)

if __name__ == '__main__':
    rospy.init_node('video')
    try:
        voice = Voice()
        rospy.loginfo("running...")
        rospy.spin()
    except rospy.ROSInterruptException: pass
