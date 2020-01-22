#!/usr/bin/env python
#
# Copyright(C) 2017 Panasonic Corporation. All Right reserved.
# 
# \file        [OuterEnvPublisher.py]
# \brief       	Main class for publish outer environment data.
# \date        2017/02/02  create a new
#
# \note        
#
#
import rospy

from SensorPublisher import SensorPublisher
from OuterEnvSerializer import OuterEnvSerializer
from ReaderFactory import ReaderFactory

class OuterEnvPublisher(SensorPublisher):
	"""Main class for publish outer environment data."""
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
		socket_name: domain socket name."""
	sp = OuterEnvPublisher()

	topic_name = []
	topic_name.append(rospy.get_param('~topic_name', 'temprature_sensor'))
	topic_name.append(rospy.get_param('~topic_name2', 'humidity_sensor'))
	topic_name.append(rospy.get_param('~topic_name3', 'air_pressure_sensor'))

	sp.loop(topic_name
		, ReaderFactory.create(), 1, OuterEnvSerializer())



