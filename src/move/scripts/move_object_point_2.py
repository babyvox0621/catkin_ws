#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PointStamped, Point, Pose
from move.msg import coordinate
from visualization_msgs.msg import Marker
import tf2_ros
import tf
import numpy as np

class MoveObjectPoint:
    def __init__(self):

        self.sub = rospy.Subscriber('pose_info', coordinate, self.callback)
        self.pub_marker = rospy.Publisher('arrow_pub', Marker, queue_size=10)
        self.is_detedted == False

    def callback(self, msg):

        self.is_detected = msg.detected
        if self.is_detected:

            # get tf
            tfBuffer1 = tf2_ros.Buffer()
            tfBuffer2 = tf2_ros.Buffer()
            listener1 = tf2_ros.TransformListener(tfBuffer1)
            listener2 = tf2_ros.TransformListener(tfBuffer2)
            try:
                trans1 = tfBuffer1.lookup_transform('base_link', 'camera_link', rospy.Time(0), rospy.Duration(3.0))
                trans2 = tfBuffer2.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(3.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(e)
            else:

            	goal = self.transform_pose(self.calc_goal(self.transform_point(msg, trans1)), trans2)
            	goal_pose = MoveBaseGoal()
            	goal_pose.target_pose.header.stamp = rospy.Time.now()
            	goal_pose.target_pose.header.frame_id = 'map'
            	goal_pose.target_pose.pose = goal

            	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            	client.wait_for_server()
            	client.send_goal(goal_pose)
            	self.set_marker(goal_pose)
            	client.wait_for_result()
            	client.get_result()
            	print(goal_pose)

    def transform_point(self, point, trans):

        p = [point.x,
             point.y,
             point.z,
             0.0]

        q = [trans.transform.rotation.x,
    		 trans.transform.rotation.y,
    		 trans.transform.rotation.z,
    		 trans.transform.rotation.w]

    	new_p = tf.transformations.quaternion_multiply(
        		tf.transformations.quaternion_multiply(q, p),
        		tf.transformations.quaternion_inverse(q))

        new_point = Point()
        new_point.x = new_p[0] + trans.transform.translation.x
        new_point.y = new_p[1] + trans.transform.translation.y
        new_point.z = new_p[2] + trans.transform.translation.z

        return new_point

    def transform_pose(self, pose, trans):

        p = [pose.position.x,
             pose.position.y,
             pose.position.z,
             0.0]

        ornt = [pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w]

        q = [trans.transform.rotation.x,
             trans.transform.rotation.y,
             trans.transform.rotation.z,
             trans.transform.rotation.w]

        new_ornt = tf.transformations.quaternion_multiply(q,ornt)
        new_p = tf.transformations.quaternion_multiply(
		        tf.transformations.quaternion_multiply(q, p),
		        tf.transformations.quaternion_inverse(q))

        new_pose = Pose()
        new_pose.position.x = new_p[0] + trans.transform.translation.x
        new_pose.position.y = new_p[1] + trans.transform.translation.y
        new_pose.position.z = new_p[2] + trans.transform.translation.z

        new_pose.orientation.x = new_ornt[0]
        new_pose.orientation.y = new_ornt[1]
        new_pose.orientation.z = new_ornt[2]
        new_pose.orientation.w = new_ornt[3]

        return new_pose

    def calc_goal(self, point):

        d = 0.3
        r = 0.05
        phi = 0
        theta = np.arctan(point.y / point.x)
        if ((point.x < 0)or((point.x == 0)&(point.y < 0))):
        	theta += np.pi

        # calc goal pose
        pose = Pose()
        pose.position.x = point.x + r*np.cos(theta) - (d + r)*np.cos(theta - phi)
        pose.position.y = point.y + r*np.sin(theta) - (d + r)*np.sin(theta - phi)
        pose.position.z = 0

        q = tf.transformations.quaternion_about_axis(theta - phi, (0,0,1))

        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        return pose


    def set_marker(self, goal_pose):
        # Set Marker data
        marker_data = Marker()
        marker_data.header.frame_id = "map"
        marker_data.header.stamp = rospy.Time.now()
        marker_data.ns = "basic_shapes"
        marker_data.id = 0
        marker_data.action = Marker.ADD
        marker_data.pose = goal_pose.target_pose.pose
        marker_data.color.r = 1.0
        marker_data.color.g = 0.0
        marker_data.color.b = 0.0
        marker_data.color.a = 1.0
        marker_data.scale.x = 0.4
        marker_data.scale.y = 0.05
        marker_data.scale.z = 0.05
        marker_data.lifetime = rospy.Duration()
        marker_data.type = 0

        # Publish goal's Marker
        self.pub_marker.publish(marker_data)

if __name__ == '__main__':
    rospy.init_node('move_object_point')
    try:
        MOP = MoveObjectPoint()
        rospy.spin()
    except rospy.ROSInterruptException: pass
