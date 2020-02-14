#!/usr/bin/env python
"""
Created on Fri Nov 29 08:59:57 2019

@author: 4035207
"""

import rospy
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

def set_marker(point):
    # Set Marker data
    marker_data = Marker()
    marker_data.header.frame_id = "optical_ranging_sensor_left"
    marker_data.header.stamp = rospy.Time.now()
    marker_data.ns = "basic_shapes"
    marker_data.id = 0
    marker_data.action = Marker.ADD
    marker_data.pose.position = point
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
    pub = rospy.Publisher('object_point', PointStamped, queue_size=10)
    rospy.init_node('detection', anonymous=True)
    r = rospy.Rate(10)
    
    msg = PointStamped()
    msg.header.frame_id = 'optical_ranging_sensor_left'
    msg.point.x = 1
    msg.point.y = 1
    msg.point.z = 0
    
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        rospy.loginfo(msg)
        pub.publish(msg)
        set_marker(msg.point)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
        
