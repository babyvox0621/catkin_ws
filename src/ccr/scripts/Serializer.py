#!/usr/bin/env python
#
# Copyright(C) 2017 Panasonic Corporation. All Right reserved.
# 
# \file        [Serializer.py]
# \brief       	Serialize/Deserialize Sensor data class.
# \date        2017/02/02  create a new
#
# \note        
#
#
from abc import ABCMeta, abstractmethod

class Serializer():
	"""Serialize/Deserialize Sensor data class."""
	__metaclass__=ABCMeta

	@abstractmethod
	def getSize(self):
		"""return the size to be deserialized."""
		pass

	@abstractmethod
	def getMessageType(self):
		"""return Class type to be published."""
		pass

	@abstractmethod
	def serialize(self, p_content):
		"""serialize Class Object to IPC serialized data."""
		pass

	@abstractmethod
	def deserialize(self, p_content):
		"""deserialize IPC serialized data to Class Object to be published."""
		pass

