#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from move.msg import coordinate
from visualization_msgs.msg import Marker
import numpy as np

def set_marker(point):
    # Set Marker data
    marker_data = Marker()
    marker_data.header.frame_id = "camera_link"
    marker_data.header.stamp = rospy.Time.now()
    marker_data.ns = "basic_shapes"
    marker_data.id = 0
    marker_data.action = Marker.ADD
    marker_data.pose.position.x = point.x
    marker_data.pose.position.y = point.y
    marker_data.pose.position.z = point.z
    marker_data.color.r = 1.0
    marker_data.color.g = 0.0
    marker_data.color.b = 0.0
    marker_data.color.a = 1.0
    marker_data.scale.x = 0.05
    marker_data.scale.y = 0.05
    marker_data.scale.z = 0.05
    marker_data.lifetime = rospy.Duration()
    marker_data.type = 2

    # Publish Marker
    pub_marker = rospy.Publisher('object', Marker, queue_size=10)
    pub_marker.publish(marker_data)

def talker():
    pub = rospy.Publisher('pose_info', coordinate, queue_size=10)
    rospy.init_node('merge', anonymous=True)
    r = rospy.Rate(20)

    msg = coordinate()
    #msg.header.frame_id = 'camera_link'
    msg.x = -0.3
    msg.y = 0
    msg.z = 0.5
    msg.detected = False

    cnt_th = 200 # cnt_thを超えると物体を検出した信号を送信
    cnt = 0

    while not rospy.is_shutdown():
        #msg.header.stamp = rospy.Time.now()
        if cnt > cnt_th:
            msg.detected = True
        # depth sensorの誤差を再現
        msg.x = -0.3#0.95 + np.random.randn()*0.1
        msg.y = 0.0#0.95 + np.random.randn()*0.1
        msg.z = 0.5
        rospy.loginfo(msg)
        pub.publish(msg)
        set_marker(msg)
        r.sleep()
        cnt += 1

if __name__ == '__main__':
	try:
		talker()
		rospy.spin()
	except rospy.ROSInterruptException: pass
