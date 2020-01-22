#!/usr/bin/env python
#
# Copyright(C) 2017 Panasonic Corporation. All Right reserved.
# 
# \file        [AirConditionSerializer.py]
# \brief       	Serialize/Deserialize Air conditaion data.
#				Current actual data is PM2.5 data.
# \date        2017/02/02  create a new
#
# \note        
#
#

from Serializer import Serializer

from ccr_msgs.msg import AirCondition
import CommonConfig
import rospy

import struct

class AirConditionSerializer(Serializer):
	"""
	Serialize/Deserialize Air conditaion data.
	Current actual data is PM2.5 data.
	"""
	def getSize(self):
		""" see Serializer."""
#		return 3
		return 8

	def getMessageType(self):
		""" see Serializer."""
		return AirCondition

	def serialize(self, p_content):
		""" see Serializer."""
#		return struct.pack('BBB', p_content.mem1, p_content.mem2, p_content.mem3)
		return struct.pack('HHHH', p_content.mem1, p_content.mem2, p_content.mem3, p_content.mem4)

	def deserialize(self, p_content):
		""" see Serializer."""
#		if len(p_content) < 3:
		if len(p_content) < 8:
			return None

		msg = AirCondition()
#		org = struct.unpack('BBB', p_content)
		org = struct.unpack('HHHH', p_content)
		msg.mem1 = org[0]
		msg.mem2 = org[1]
		msg.mem3 = org[2]
		msg.mem4 = org[3]

		if CommonConfig.DEBUG_FLAG is True:
			rospy.logdebug( 'mem1=' + str(msg.mem1))
			rospy.logdebug( 'mem2=' + str(msg.mem2))
			rospy.logdebug( 'mem3=' + str(msg.mem3))
			rospy.logdebug( 'mem4=' + str(msg.mem4))

		return msg

