#!/usr/bin/env python
#
# Copyright(C) 2017 Panasonic Corporation. All Right reserved.
# 
# \file        [SensorPublisher.py]
# \brief       	Sensor data publishment super class.
# \date        2017/02/02  create a new
#
# \note        
#
#
from abc import ABCMeta, abstractmethod

import rospy

from std_msgs.msg import String

import CommonConfig


class SensorPublisher():
	"""Sensor data publishment super class."""
	__metaclass__=ABCMeta

	def __init__(self):
		"""Constructor."""
		rospy.init_node("default_sensor_node", anonymous=True)
		

	@abstractmethod
	def preRead(self):
		""" use it if something to be done before reading data from daemon's interface."""
		pass

	@abstractmethod
	def postRead(self):
		""" use it if something to be done after reading data from daemon's interface."""
		pass

	def loop(self, p_topicName, p_reader, p_hz, p_serializer):
		""" data reading and publishment main loop."""
		types = p_serializer.getMessageType()

		if types is None:
			pass
		elif isinstance(types, list):
			pub = []
			for i in range(0, len(types)):
				pub.append(rospy.Publisher(p_topicName[i], types[i], queue_size=1))
		else:
			pub = rospy.Publisher(p_topicName,types , queue_size=1)

		r = rospy.Rate(p_hz)

		try:
			p_reader.connect()

			while not rospy.is_shutdown():
				if CommonConfig.DEBUG_FLAG is True:
					rospy.logdebug( '---in loop---')

				self.preRead()
				content = p_reader.get(p_serializer.getSize())
				self.postRead()

				#print content

				msgs = p_serializer.deserialize(content)
				if msgs is None:
					pass
				elif isinstance(msgs, list):
					for i in range(0, len(msgs)):
						pub[i].publish(msgs[i])
				else:
					pub.publish(msgs)

				r.sleep()
		except Exception as e:
			rospy.logerr( '---exception raise---')
			rospy.logerr( str(type(e)) + str(e.args) + e.message)

		print 'loop end.'

		p_reader.close()
