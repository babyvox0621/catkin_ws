#!/usr/bin/env python
#
# Copyright(C) 2017 Panasonic Corporation. All Right reserved.
# 
# \file        [SystemInfoSerializer.py]
# \brief       	Serialize/Deserialize System Information.
# \date        2017/02/02  create a new
#
# \note        
#
#
from Serializer import Serializer
from Math import Math

from ccr_msgs.msg import SystemInfo
import CommonConfig

import struct
import rospy

class SystemInfoSerializer(Serializer):
	"""Serialize/Deserialize System Information."""
	def getSize(self):
		""" see Serializer."""
		return 12

	def getMessageType(self):
		""" see Serializer."""
		return SystemInfo

	def serialize(self, p_content):
		""" see Serializer."""
		pass

	def deserialize(self, p_content):
		""" see Serializer."""
#   if len(p_content) < getSize():  ?
		if len(p_content) < 12:
			return None

		msg = SystemInfo()

#		org = struct.unpack('fff', p_content)
#
#		msg.header.stamp = rospy.Time.now()
#		msg.voltage = Math.roundFloatToDouble(org[0])
#		msg.inner_temperature = Math.roundFloatToDouble(org[1])
#		msg.lte_strength = Math.roundFloatToDouble(org[2])

		org = struct.unpack('hhhhhh', p_content)

		msg.header.stamp = rospy.Time.now()
		msg.lte_strength = org[0]
		msg.ethernet_link = org[1]
		msg.wifi_strength = org[2]
		msg.inner_temperature = float(org[3]) / 100.0;
		msg.battery = float(org[4]) / 100.0
		msg.voltage = float(org[5]) / 100.0

		if CommonConfig.DEBUG_FLAG is True:
			rospy.logdebug( 'wifi=' + str(msg.wifi_strength))
			rospy.logdebug( 'ethernet' + str(msg.ethernet_link))
			rospy.logdebug( 'voltage=' + str(msg.voltage) + "V")
			rospy.logdebug( 'battery=' + str(msg.battery) + "%")
			rospy.logdebug( 'inner_temperature=' + str(msg.inner_temperature))
			rospy.logdebug( 'lte_strength=' + str(msg.lte_strength))

		return msg

