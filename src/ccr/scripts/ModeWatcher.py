#!/usr/bin/env python
#
# Copyright(C) 2017 Panasonic Corporation. All Right reserved.
# 
# \file        [ModeWatcher.py]
# \brief       watch STM mode and anounce it.
# \date        2017/05/24  create a new
#
# \note        
#
#
import threading

import rospy
import tf
from std_msgs.msg import Byte
import pygame.mixer

import CommonConfig

LOOP_TIMER = 1.0

class ModeWatcher():
	"""watch STM mode and anounce it."""

	def __init__(self):
		"""constructor"""
		self.m_alive = False
		self.m_prev_mode = 0
		pygame.mixer.init()

	def start(self):
		"""start to subscribe/publish ."""
		if CommonConfig.DEBUG_FLAG is True:
			rospy.logdebug( "ModeWatcher#start")

		sub_topic_name = rospy.get_param("~sub_topic", "mobile_base/event/mode")
		self.sub = rospy.Subscriber(sub_topic_name, Byte , self.onMode)

		self.m_timer = threading.Timer(LOOP_TIMER, self.asyncLoop)
		self.m_timer.start()

		self.m_alive = True

	def finish(self):
		"""stop to subscribe/publish"""
		pygame.mixer.music.stop()
		self.m_timer.cancel()
		self.m_alive = False

	def onMode(self, mode):
		#print("mode is %d" % mode.data)
		""" check force stop mode."""
		if mode.data is not self.m_prev_mode:
			if mode.data is 3:
				if CommonConfig.DEBUG_FLAG is True:
					rospy.logdebug( "ModeWatcher#onMode force stop!")
				#print("force stop!")
				pygame.mixer.music.load('/home/ubuntu/catkin_ws/src/ccr/scripts/stop.mp3')
				pygame.mixer.music.set_volume(0.5)
				pygame.mixer.music.play(1)
			if mode.data is 1 and self.m_prev_mode is 3:
				if CommonConfig.DEBUG_FLAG is True:
					rospy.logdebug( "ModeWatcher#onMode force stop recover!")
				#print("force stop!")
				pygame.mixer.music.load('/home/ubuntu/catkin_ws/src/ccr/scripts/restart.mp3')
				pygame.mixer.music.set_volume(0.5)
				pygame.mixer.music.play(1)

		""" store current mode."""
		self.m_prev_mode = mode.data

	def asyncLoop(self):
		pass
		
		# set next timer.
		if self.m_alive is True:
			self.m_timer = threading.Timer(LOOP_TIMER, self.asyncLoop)
			self.m_timer.start()


if __name__ == "__main__":
	"""
	watch STM mode and anounce it.
	"""
	# initialize as ROS node.
	rospy.init_node("mode_watcher", anonymous=True)

	# gather IoT device information.
	idc = ModeWatcher()
	idc.start()

	rospy.spin()

	idc.finish()

