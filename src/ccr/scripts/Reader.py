#!/usr/bin/env python
#
# Copyright(C) 2017 Panasonic Corporation. All Right reserved.
# 
# \file        [Reader.py]
# \brief       	Abstract accessor for sensor daemon.
# \date        2017/02/02  create a new
#
# \note        
#
#
from abc import ABCMeta, abstractmethod

import rospy

from std_msgs.msg import String


class Reader():
	"""Abstract accessor for sensor daemon."""
	__metaclass__=ABCMeta

	@abstractmethod
	def connect(self):
		"""connect to daemon's interface."""
		pass

	@abstractmethod
	def close(self):
		"""close daemon's interface."""
		pass

	@abstractmethod
	def get(self, p_size):
		"""get daemon's data.
			Args:
				p_size: to be read data's size.
		"""
		pass


