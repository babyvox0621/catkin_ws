#!/usr/bin/env python
#
# Copyright(C) 2017 Panasonic Corporation. All Right reserved.
# 
# \file        [ReaderFactory.py]
# \brief       	Factory class for Reader class.
# \date        2017/02/02  create a new
#
# \note        
#
#
import rospy

from SocketReader import SocketReader

SOCKET_PATH = "/tmp/socket_test"

class ReaderFactory():
	"""Factory class for Reader class."""
	@staticmethod
	def create():
		"""return suitable Reader instance for current environment."""
		socket_name = rospy.get_param('~socket_name', SOCKET_PATH)
		#socket_name = "/tmp/out_env_sensor.sock"
		return SocketReader(socket_name)



