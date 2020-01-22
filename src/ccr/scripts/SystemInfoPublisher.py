#!/usr/bin/env python
#
# Copyright(C) 2017 Panasonic Corporation. All Right reserved.
# 
# \file        [SystemInfoPublisher.py]
# \brief       	Main class for publish system information.
# \date        2017/02/02  create a new
#
# \note        
#
#
import rospy

from SensorPublisher import SensorPublisher
from SystemInfoSerializer import SystemInfoSerializer
from ReaderFactory import ReaderFactory

class SystemInfoPublisher(SensorPublisher):
	"""Main class for publish system information."""
	def preRead(self):
		return

	def postRead(self):
		return

if __name__ == "__main__":
	"""
	do publish sensor data.
	need parameter(rosparam):
		topic_name : publising topic name.
		socket_name: domain socket name.
	"""
	sp = SystemInfoPublisher()

	topic_name = rospy.get_param('~topic_name', 'mobile_base/event/system_info')

	sp.loop(topic_name
		, ReaderFactory.create(), 1, SystemInfoSerializer())



