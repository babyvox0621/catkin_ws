#!/usr/bin/env python
#
# Copyright(C) 2017 Panasonic Corporation. All Right reserved.
# 
# \file        [SocketReader.py]
# \brief       	Socket accessor for sensor daemon.
# \date        2017/02/02  create a new
#
# \note        
#
#
import os
import socket

import rospy

from std_msgs.msg import String

#local reference
from Reader import Reader

class SocketReader(Reader):
	"""Socket accessor for sensor daemon."""
	def __init__(self, p_path):
		""" constructor.
			Args:
				path: unix domain sokect's path.
		"""
		self.path = p_path
		rospy.logdebug( "!---------------------------------")

	def connect(self):
		"""connect to daemon's interface."""
		self.s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

		rospy.logdebug( "path is" + self.path)
		rospy.logdebug( "---------------------------------")

		self.s.connect(self.path)

	def get(self, p_size):
		"""get daemon's data."""
		return self.s.recv(p_size)

	def close(self):
		"""close daemon's interface."""
		self.s.close();

