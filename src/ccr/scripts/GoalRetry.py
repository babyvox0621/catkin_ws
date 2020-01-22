#!/usr/bin/env python
#
# Copyright(C) 2017 Panasonic Corporation. All Right reserved.
# 
# \file        [GoalRetry.py]
# \brief       	retry goal setting.
# \date        2017/04/20  create a new
#
# \note        
#
#
import threading

import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from std_srvs.srv import Empty
from actionlib_msgs.msg import GoalStatus

class GoalRetry():
	"""Notify the robot is near by the goal."""

	def __init__(self):
		"""constructor"""
		self.m_true_goal = None
		self.m_alive = False

	def start(self):
		"""start to subscribe/publish ."""
		rospy.logdebug( "GoalRetry#start")

		sub_topic_name = rospy.get_param("~sub_topic", "move_base_simple/goal")
		self.sub = rospy.Subscriber(sub_topic_name, PoseStamped , self.onSetGoal)
		self.sub2 = rospy.Subscriber("/move_base/result", MoveBaseActionResult , self.onGoalStatus)

		self.map_frame = rospy.get_param("~map_frame", "map")
		self.body_frame = rospy.get_param("~body_frame", "base_footprint")

		self.pub = rospy.Publisher("/move_base_simple/goal",PoseStamped , queue_size=1)

		self.clear = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)

		self.m_alive = True
		#self.tf_listener = tf.TransformListener()
		self.m_timer = threading.Timer(1.0, self.asyncLoop)
		self.m_timer.start()

	def finish(self):
		"""stop to subscribe/publish"""
		self.m_alive = False
		self.m_timer.cancel()

	def onSetGoal(self, goal):
		""" store current true goal."""
		self.m_true_goal = goal
		rospy.logdebug( "GoalRetry#onSetGoal[%f][%f]" % (goal.pose.position.x, goal.pose.position.y))

	def onGoalStatus(self, result):
		""" robot successfly reached to goal?"""
		rospy.logdebug( "GoalRetry#onGoalStatus[%d]" % (result.status.status))

		if result.status.status in [4,5,6,9]:
			self.clear()
			self.pub.publish(self.m_true_goal)

	def asyncLoop(self):
		pass


if __name__ == "__main__":
	"""
	retry goal setting.
	"""
	# initialize as ROS node.
	rospy.init_node("near_goal_publisher", anonymous=True)

	ng = GoalRetry()
	ng.start()

	rospy.spin()

	ng.finish()

