#!/usr/bin/env python
# coding: utf-8
#
# Copyright(C) 2017 Panasonic Corporation. All Right reserved.
# 
# \file        [OdomCalibration.py]
# \brief       	Calibrate wheel axis and radius.
# \date        2017/07/3  create a new
#
# \note        
#
#
import threading

import rospy
import tf
import math
import time
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionResult
from std_srvs.srv import Empty
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from ccr_msgs.msg import Drive
from geometry_msgs.msg import Quaternion
from rosparam import upload_params
import yaml
from yaml import load

class OdomCalibration():
	"""Calibrate wheel axis and radius."""

	STATE_INIT = 0x00
	STATE_RADIUS = 0x01
	STATE_AXIS = 0x02

	def __init__(self):
		"""constructor"""
		self.m_true_goal = None
		self.m_alive = False
		self.cmd_vel = Twist()
		self.m_encoder_left = 0
		self.m_encoder_right = 0
		self.m_imu = None
		self.m_scan = None

		self.m_state = OdomCalibration.STATE_INIT
		self.yaml_path = "/home/ubuntu/catkin_ws/src/param/ccr/param/encoder.yaml"

	def start(self):
		"""start to subscribe/publish ."""
		rospy.logdebug( "OdomCalibration#start")

		#self.m_encoder_cycle = rospy.get_param("~max_cycle_encoder_counts", 300)
		# store and set data.
		f = open(self.yaml_path, 'r')
		yamlfile = load(f)
		f.close()
		self.m_encoder_cycle = yamlfile["max_cycle_encoder_counts"]

		self.sub = rospy.Subscriber("odom", Odometry , self.onOdom)
		self.sub2 = rospy.Subscriber("imu", Imu , self.onImu)
		self.sub3 = rospy.Subscriber("scan", LaserScan , self.onScan)
		self.sub3 = rospy.Subscriber("mobile_base/event/drive", Drive , self.onDrive)
		#self.sub3 = rospy.Subscriber("drive", Drive , self.onDrive)
		self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

		s1 = 'Please set LetsBot in front of the wall(apart from 1m).\n And press Enter key.'
		print s1
		raw_input(">>>")

		# sensor check.
		if self.m_imu is None:
			print 'IMU does not work. finish.'
			return

		if self.m_scan is None:
			print 'LiDAR does not work. finish.'
			return


		self.m_range_at_first = self.getRange()
		self.m_encoder_left_at_first = self.m_encoder_left
		self.m_encoder_right_at_first = self.m_encoder_right

		# at first, get wheel radius!
		self.m_state = OdomCalibration.STATE_RADIUS

		# go strait!!
		cmd_vel = Twist()
		cmd_vel.linear.x = 0.2
		self.cmd_vel = cmd_vel

		self.m_alive = True
		#self.tf_listener = tf.TransformListener()
		self.m_timer = threading.Timer(1.0, self.asyncLoop)
		self.m_timer.start()

	def finish(self):
		"""stop to subscribe/publish"""
		self.cmd_vel = Twist()
		time.sleep(1)

		self.m_alive = False
		self.m_timer.cancel()

	def onImu(self, imu):
		""" got sensor data."""
		self.m_imu = imu

		if self.m_state is OdomCalibration.STATE_AXIS:
			q1 = Quaternion()
			q1.x = self.m_imu_at_first.orientation.x
			q1.y = self.m_imu_at_first.orientation.y
			q1.z = self.m_imu_at_first.orientation.z
			q1.w = self.m_imu_at_first.orientation.w

			q2 = Quaternion()
			q2.x = imu.orientation.x
			q2.y = imu.orientation.y
			q2.z = imu.orientation.z
			q2.w = imu.orientation.w

			# check turning 1 cycle?
			yaw1 = tf.transformations.euler_from_quaternion((q1.x, q1.y, q1.z, q1.w))
			yaw2 = tf.transformations.euler_from_quaternion((q2.x, q2.y, q2.z, q2.w))
			diff = yaw1[2] - yaw2[2]

			if abs(diff) < 0.1: # return to first degree?
				# stop!
				self.cmd_vel = Twist()
				time.sleep(1)
				
				# how did wheels rotate?
				left = self.m_encoder_left - self.m_encoder_left_at_first
				right = self.m_encoder_right - self.m_encoder_right_at_first

				left_rot = float(left) / self.m_encoder_cycle
				right_rot = float(right) / self.m_encoder_cycle
				rospy.logdebug("encoder left=%f right=%f" % (left, right))

				sin_diff = (right_rot * self.radius) - (left_rot * self.radius)

				# sin_diff / wheel_separation_ = angular ==> wheel_separation = sin_diff / angular
				wheel_separation = sin_diff / 2 * math.pi
				rospy.loginfo("estimated axis diff=%f" % wheel_separation)
				self.wheel_separation = wheel_separation

				# store and set data.
				f = open(self.yaml_path, 'r')
				yamlfile = load(f)
				f.close()
				rospy.loginfo("org param:" + yaml.dump(yamlfile, default_flow_style=False))

				yamlfile["wheel_diameter"] = (self.radius * 2)
				yamlfile["axle_length"] = self.wheel_separation
				rospy.set_param("/ccrdriver/wheel_diameter", self.radius * 2)
				rospy.set_param("/ccrdriver/axle_length", self.wheel_separation)

				f = open(self.yaml_path, 'w')
				f.write(yaml.dump(yamlfile, default_flow_style=False))
				f.close()

				# end.
				self.m_state = OdomCalibration.STATE_INIT
				self.m_alive = False
				#time.sleep(1)
				#rospy.signal_shutdown("Calibration Success!!")
				rospy.loginfo("Calibration Success!! please press Ctrl+C. ")

	def onScan(self, scan):
		""" got sensor data."""
		self.m_scan = scan

		if self.m_state is OdomCalibration.STATE_RADIUS:
			#rospy.loginfo("move " + str( self.m_range_at_first - self.getRange()))
			# check running 1m?
			if self.m_range_at_first - self.getRange() > 1.0:
				# stop!
				self.cmd_vel = Twist()
				time.sleep(1)

				# how did wheels rotate?
				left = self.m_encoder_left - self.m_encoder_left_at_first
				right = self.m_encoder_right - self.m_encoder_right_at_first

				left_rot = float(left) / self.m_encoder_cycle
				right_rot = float(right) / self.m_encoder_cycle
				rospy.logdebug("encoder left=%f right=%f" % (left, right))

				# rotation num * 2 * PI * radius = 1.0 ==> radius = 1 / (rotation * 2 * PI)
				left_radius = 1.0 / (left_rot * 2 * math.pi)
				right_radius = 1.0 / (right_rot * 2 * math.pi)

				# average and cut.
				radius = (left_radius + right_radius) / 2
				self.radius = round(radius, 3)
				rospy.loginfo("estimated radius=%f" % self.radius)

				# next, get axis!!
				self.m_imu_at_first = self.m_imu
				self.m_encoder_left_at_first = self.m_encoder_left
				self.m_encoder_right_at_first = self.m_encoder_right

				self.cmd_vel.angular.z = 0.2
				time.sleep(3)

				self.m_state = OdomCalibration.STATE_AXIS

	def onOdom(self, odom):
		""" got sensor data."""
		self.m_odom = odom

	def onDrive(self, drive):
		""" got sensor data."""
		self.m_encoder_left += drive.left_encoder_count
		self.m_encoder_right += drive.right_encoder_count

	def getRange(self):
		cur = self.m_scan.angle_min
		cnt = 0
		# get front angle offset.
		while cur <= 0:
			cur += self.m_scan.angle_increment
			cnt += 1

		return self.m_scan.ranges[cnt]

	def asyncLoop(self):
		if self.m_alive is False:
			return

		self.pub.publish(self.cmd_vel)
		self.m_timer = threading.Timer(0.1, self.asyncLoop)
		self.m_timer.start()


if __name__ == "__main__":
	"""
	Calibrate wheel axis and radius.
	"""
	# initialize as ROS node.
	rospy.init_node("odom_calibration", anonymous=True)

	ng = OdomCalibration()
	ng.start()

	rospy.spin()

	ng.finish()

