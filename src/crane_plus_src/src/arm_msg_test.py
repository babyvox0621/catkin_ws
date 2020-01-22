#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8


def test_msg():
	rospy.init_node('test_msg')
	pub = rospy.Publisher('arm_msg', Int8, queue_size=10)
	msg = 0
	
	r = rospy.Rate(10)
	while not rospy.is_shutdown():
		#grab
		msg = 1
		pub.publish(msg)
		r.sleep()

		#release
		msg = 2
		pub.publish(msg)
		r.sleep()

if __name__=='__main__':
	try:
		tm = test_msg()
	except rospy.ROSInterruptException:
		pass
