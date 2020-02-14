#!/usr/bin/env python
import paho.mqtt.client as mqtt
import json
import subprocess
import rospy
import subprocess
import time
from move.msg import commond

class Voice:

    def __init__(self):

        self.sub_video = rospy.Subscriber('Commond_TOP', commond, self.callback)
        self.sub_video = rospy.Subscriber('results_GOOGLE', commond, self.callbackGoogle)
        self.pub_video = rospy.Publisher('results_VOICE', commond, queue_size=1)
        self.filename = 'cigarette_cm.mp4'
        self.flag1 = 0
        self.flag2 = 0
        self.flag3 = 0
        rospy.loginfo('init finished')

    def on_connect(self, client, userdata, flags, respons_code):
        print('status {0}'.format(respons_code))
        client.subscribe(TOPIC)

    def on_message(self, client, userdata, msg):
        data = json.loads(msg.payload.decode("utf-8"))["data"]
        print(data)
        self.flag=1

    def callbackGoogle(self, msg):
        if((msg.node == 4)or(msg.node == 0)):
            if(msg.msg.id == 1):
                self.flag1=1
            else(msg.msg.id == 2):
                self.flag2=1

    def callback(self, msg):
        if((msg.node == 4)or(msg.node == 0)):
            send = commond()
            send.topic_id = msg.topic_id
            rospy.loginfo("%d", msg.topic_id)
            if(msg.msg.id == 0):
                rospy.loginfo("waiting...")
                send.ret = 0
                self.flag3=1;

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
        while(1):
            rospy.spinonce()
            if(self.flag1==1&&self.flag3==1):
                topic send


    except rospy.ROSInterruptException: pass
