#!/usr/bin/env python
#
# Copyright(C) 2017 Panasonic Corporation. All Right reserved.
# 
# \file        [OuterEnvSerializer.py]
# \brief       	Serialize/Deserialize Outer Environment.
# \date        2017/02/02  create a new
#
# \note        
#
#
from Serializer import Serializer
from Math import Math

from sensor_msgs.msg import Temperature
from sensor_msgs.msg import RelativeHumidity
from sensor_msgs.msg import FluidPressure
import CommonConfig

import struct
import rospy

class OuterEnvSerializer(Serializer):
	"""Serialize/Deserialize Outer Environment."""
	def getSize(self):
		""" see Serializer."""
		return 12

	def getMessageType(self):
		""" see Serializer."""
		return [Temperature, RelativeHumidity, FluidPressure]

	def serialize(self, p_content):
		""" see Serializer."""
		pass

	def deserialize(self, p_content):
		""" see Serializer."""
		if len(p_content) < 12:
			return None

		msg1 = Temperature()
		msg2 = RelativeHumidity()
		msg3 = FluidPressure()

		org = struct.unpack('fff', p_content)

		msg1.header.stamp = rospy.Time.now()
		# round float value to double.
		msg1.temperature = Math.roundFloatToDouble(org[0])
		msg1.variance = 0

		msg2.header.stamp = rospy.Time.now()
		msg2.relative_humidity = Math.roundFloatToDouble(org[1])
		msg2.variance = 0

		msg3.header.stamp = rospy.Time.now()
		msg3.fluid_pressure = Math.roundFloatToDouble(org[2])
		msg3.variance = 0

		msgs = [msg1, msg2, msg3]

		if CommonConfig.DEBUG_FLAG is True:
			rospy.logdebug( 'temperature=' + str(msg1.temperature))
			rospy.logdebug( 'humidity=' + str(msg2.relative_humidity))
			rospy.logdebug( 'pressure=' + str(msg3.fluid_pressure))

		return msgs

