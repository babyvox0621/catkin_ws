#!/usr/bin/env python
#
# Copyright(C) 2017 Panasonic Corporation. All Right reserved.
# 
# \file        [GpsSerializer.py]
# \brief       		Serialize/Deserialize GPS location data.
#	this class uses 3rd party library libnmea_navsat_driver.
#	data publishment is done in the library, so this class doesn't
#	publish data.
#
# \note        
#
#
from Serializer import Serializer

import CommonConfig

import rospy
import libnmea_navsat_driver.driver

class GpsSerializer(Serializer):
	"""
	Serialize/Deserialize GPS location data.
	this class uses 3rd party library libnmea_navsat_driver.
	data publishment is done in the library, so this class doesn't
	publish data."""
	def __init__(self):
		"""constructor."""
		node_name = rospy.get_param('topic_name', 'mobile_base/event/gps/')

		self.driver = libnmea_navsat_driver.driver.RosNMEADriver(node_name)
		self.frame_id = libnmea_navsat_driver.driver.RosNMEADriver.get_frame_id()
		self.leftPart = None

	def getSize(self):
		""" see Serializer."""
		return 256

	def getMessageType(self):
		""" see Serializer."""
		# Message is published in nmea driver.
		return None

	def serialize(self, p_content):
		""" see Serializer."""
		pass

	def deserialize(self, p_content):
		""" see Serializer."""
		if len(p_content) is 0:
			return None

		try:
			if CommonConfig.DEBUG_FLAG is True:
				rospy.logdebug( "---> GPS raw data---")
				rospy.logdebug( p_content)
				rospy.logdebug( "<--- GPS raw data---")
			lines = p_content.split('\r\n')
			isContinue = False
			leftPart = self.leftPart 
			if p_content[len(p_content) - 1] is not '\n':
				isContinue = True
			elif p_content[len(p_content) - 1] is not '\r':
				isContinue = True
			else:
				self.leftPart = None
			for i in range(0 , len(lines)):
				line = lines[i]
				rospy.logdebug(line)
				if len(line) is 0:
					self.leftPart = None
					leftPart = None
					continue
				if line[0] is '\r':
					self.leftPart = None
					leftPart = None
					continue
				if line[0] is '\n':
					self.leftPart = None
					leftPart = None
					continue
				if isContinue is True:
					if i == len(lines) -1:
						self.leftPart = lines[len(lines) - 1]
						continue
				if self.leftPart != None:
					line = leftPart + line
					self.leftPart = None
				self.driver.add_sentence(line, self.frame_id)
		except ValueError as e:
			rospy.logdebug( 'GPS data is invalid format.')
			rospy.logdebug( str(type(e)) + str(e.args) + e.message)

		if CommonConfig.DEBUG_FLAG is True:
			rospy.logdebug( 'GPS driver sequence done.')

		return None

