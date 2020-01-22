#!/usr/bin/env python
#
# Copyright(C) 2017 Panasonic Corporation. All Right reserved.
# 
# \file        [Status.py]
# \brief       	check battery status.
# \date        2017/04/25  create a new
#
# \note        
#
#
import threading

import rospy
import tf
from std_msgs.msg import String
from ccr_msgs.msg import Battery

class BatteryStatus():
	"""check battery status."""

	def __init__(self):
		"""constructor"""

	def start(self):
		"""start to subscribe/publish ."""
		rospy.logdebug( "BatteryStatus#start")

		self.sub = rospy.Subscriber("mobile_base/event/battery", Battery, self.onBattery)

		self.pub = rospy.Publisher("/Led",String , queue_size=1)

		self.m_alive = True
		self.m_timer = threading.Timer(1.0, self.asyncLoop)
		self.m_timer.start()
		self.before_time = rospy.Time.now()

	def finish(self):
		"""stop to subscribe/publish"""
		self.m_alive = False
		self.m_timer.cancel()

	def onBattery(self, battery):
		""" check battery status."""
		if rospy.Time.now() - self.before_time < rospy.Duration(3.0):
			return

		self.before_time = rospy.Time.now()

		#if battery.voltage < 1.95 :
		if battery.voltage < 23 :
			pubData = String()
			pubData.data = "y\n"
			self.pub.publish(pubData)
		elif battery.voltage < 22 :
			pubData = String()
			pubData.data = "R\n"
			self.pub.publish(pubData)

	def asyncLoop(self):
		pubData = String()
		pubData.data = "X\n"
		self.pub.publish(pubData)


if __name__ == "__main__":
	"""
	check battery status.
	"""
	# initialize as ROS node.
	rospy.init_node("check_battery_status", anonymous=True)

	ng = BatteryStatus()
	ng.start()

	rospy.spin()

	ng.finish()

