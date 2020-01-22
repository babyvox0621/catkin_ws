#!/usr/bin/env python
#
# Copyright(C) 2017 Panasonic Corporation. All Right reserved.
# 
# \file        [GpsPublisher.py]
# \brief       	Main class for publish GPS location.
# \date        2017/02/02  create a new
#
# \note        
#
#
import rospy

from SensorPublisher import SensorPublisher
from GpsSerializer import GpsSerializer
from ReaderFactory import ReaderFactory

class GpsPublisher(SensorPublisher):
	"""Main class for publish GPS location."""
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
			socket_name: domain socket name.
	"""
	sp = GpsPublisher()

	sp.loop(None
		, ReaderFactory.create(), 10, GpsSerializer())



