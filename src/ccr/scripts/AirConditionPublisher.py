#!/usr/bin/env python
#
# Copyright(C) 2017 Panasonic Corporation. All Right reserved.
# 
# \file        [AirConditionPublisher.py]
# \brief       	Main class for publish air condition.
# \date        2017/02/02  create a new
#
# \note        
#
#
import rospy

from SensorPublisher import SensorPublisher
from AirConditionSerializer import AirConditionSerializer
from ReaderFactory import ReaderFactory

class AirConditionPublisher(SensorPublisher):
	"""Main class for publish air condition."""
	def preRead(self):
		"""see SensorPublisher."""
		return

	def postRead(self):
		"""see SensorPublisher."""
		return

if __name__ == "__main__":
	"""
	do publish sensor data.
	need parameter(rosparam):
		topic_name : publising topic name.
		socket_name: domain socket name.
	"""

	sp = AirConditionPublisher()

	topic_name = rospy.get_param('~topic_name', 'air_conditaion_sensor')

	sp.loop(topic_name
		, ReaderFactory.create(), 2, AirConditionSerializer())



