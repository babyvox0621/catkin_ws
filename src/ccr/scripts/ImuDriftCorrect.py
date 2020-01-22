#!/usr/bin/env python
#
# Copyright(C) 2017 Panasonic Corporation. All Right reserved.
# 
# \file        [ImuDriftCorrect.py]
# \brief       	correct imu drifting by using amcl estimation.
# \date        2017/03/23  create a new
#
# \note        
#
#
import threading

import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty
import PyKDL

import CommonConfig

LOOP_TIMER = 1.0

class ImuDriftCorrect():
	"""correct imu drifting by using amcl estimation."""

	def __init__(self):
		"""constructor"""
		self.m_alive = False
		self.rot_diff = Quaternion()
		self.pre_rot = Quaternion()
		self.tf_listener = tf.TransformListener()

	def start(self):
		"""start to subscribe/publish ."""
		if CommonConfig.DEBUG_FLAG is True:
			rospy.logdebug( "ImuDriftCorrect#start")

		self.map_frame = rospy.get_param("~map_frame", "map")
		self.odom_frame = rospy.get_param("~body_frame", "odom")

		pub_topic_name = rospy.get_param("~pub_topic", "imu_data")
		self.pub = rospy.Publisher(pub_topic_name, Imu , queue_size=1)

		sub_topic_name = rospy.get_param("~sub_topic", "amcl_pose")
		self.sub = rospy.Subscriber(sub_topic_name, PoseWithCovarianceStamped , self.onPose)

		sub_topic_name2 = rospy.get_param("~sub_topic2", "imu")
		self.sub2 = rospy.Subscriber(sub_topic_name2, Imu , self.onImu)

		sub_topic_name3 = rospy.get_param("~sub_topic3", "initialpose")
		self.sub3 = rospy.Subscriber(sub_topic_name2, PoseWithCovarianceStamped , self.onInitialPose)

		rospy.wait_for_service('/request_nomotion_update')
		self.m_nomotion_update = rospy.ServiceProxy('/request_nomotion_update', Empty)

		self.m_timer = threading.Timer(LOOP_TIMER, self.asyncLoop)
		self.m_timer.start()

		self.m_alive = True

	def finish(self):
		"""stop to subscribe/publish"""
		self.m_timer.cancel()
		self.m_alive = False

	def onPose(self, pose):
		""" store current pose and diff."""
		self.m_pre_pose = pose

		# get amcl corrected pose.
		try:
			(trans, rot) = self.tf_listener.lookupTransform(self.map_frame, self.odom_frame, rospy.Time(0))
			curQ = Quaternion()
			curQ.x = rot[0]
			curQ.y = rot[1]
			curQ.z = rot[2]
			curQ.w = rot[3]
			diff = self.decQuaternion(self.pre_rot , curQ)
			#(r, p, y) = tf.transformations.euler_from_quaternion([diff.x, diff.y, diff.z, diff.w])
			#if y < 1:
			self.rot_diff = self.addQuaternion(self.rot_diff , diff)
			self.pre_rot = rot
			if CommonConfig.DEBUG_FLAG is True:
				rospy.logdebug( "ImuDriftCorrect#onPose diff[%f][%f]" % (self.rot_diff.z, self.rot_diff.w))

		except tf.LookupException as err1:
			rospy.logerr( "[ImuDriftCorrect][ERROR]1 Cannot get TF!!" + str(err1))
		except tf.ConnectivityException as err2:
			rospy.logerr( "[ImuDriftCorrect][ERROR]2 Cannot get TF!!" + str(err2))
		except tf.ExtrapolationException as err3:
			rospy.logerr( "[ImuDriftCorrect][ERROR]3 Cannot get TF!!" + str(err3))


	def onImu(self, pose):
		"""handle imu raw data and publish corrected data."""
		pose.orientation = self.addQuaternion(pose.orientation , self.rot_diff)
		self.pub.publish(pose)

	def onInitialPose(self, pose):
		""" reset if initial pose is set."""
		self.rot_diff = Quaternion()
		self.pre_rot = Quaternion()

	def addQuaternion(self, q1, q2):
		#tf1 = PyKDL.Rotation.Quaternion(q1.x, q1.y, q1.z, q1.w)
		#tf2 = PyKDL.Rotation.Quaternion(q2.x, q2.y, q2.z, q2.w)
		#tf3 = tf2 * tf1
		#ret = Quaternion()
		#ret.x = tf3.GetQuaternion()[0]
		#ret.y = tf3.GetQuaternion()[1]
		#ret.z = tf3.GetQuaternion()[2]
		#ret.w = tf3.GetQuaternion()[3]
		yaw1 = tf.transformations.euler_from_quaternion((q1.x, q1.y, q1.z, q1.w))
		yaw2 = tf.transformations.euler_from_quaternion((q2.x, q2.y, q2.z, q2.w))
		diff = yaw1[2] + yaw2[2]
		q = tf.transformations.quaternion_from_euler(0, 0, diff)
		ret = Quaternion()
		ret.x = q[0]
		ret.y = q[1]
		ret.z = q[2]
		ret.w = q[3]
		return ret

	def decQuaternion(self, q1, q2):
		yaw1 = tf.transformations.euler_from_quaternion((q1.x, q1.y, q1.z, q1.w))
		yaw2 = tf.transformations.euler_from_quaternion((q2.x, q2.y, q2.z, q2.w))
		diff = yaw1[2] - yaw2[2]
		q = tf.transformations.quaternion_from_euler(0, 0, diff)
		ret = Quaternion()
		ret.x = q[0]
		ret.y = q[1]
		ret.z = q[2]
		ret.w = q[3]
		return ret

	def asyncLoop(self):
		
		# amcl force update
		self.m_nomotion_update()

		# set next timer.
		if self.m_alive is True:
			self.m_timer = threading.Timer(LOOP_TIMER, self.asyncLoop)
			self.m_timer.start()


if __name__ == "__main__":
	"""
	correct imu drifting by using amcl estimation.
	"""
	# initialize as ROS node.
	rospy.init_node("imu_drift_correct", anonymous=True)

	# gather IoT device information.
	idc = ImuDriftCorrect()
	idc.start()

	rospy.spin()

	idc.finish()

