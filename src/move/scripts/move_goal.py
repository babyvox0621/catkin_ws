#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PointStamped, Point, Pose
from move.msg import coordinate
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from actionlib_msgs.msg import GoalStatus
import tf2_ros
import tf
import numpy as np

'''
def callback(self, msg):

    self.is_detected = msg.detected
    rospy.loginfo('detected :', self.is_detected)

    if (self.is_detected)&(not self.is_finished):
        rospy.loginfo('Recieved coordinate : [%5.3f, %5.3f, %5.3f]', msg.x, msg.y, msg.z)
        rospy.loginfo('Start moving object pos...')
        result = self.move_goal(msg)
        if result:
            self.is_finished = True
            self.pub_finished.publish(True)
'''
class MoveCoordinate:
    """
    カメラで検出したオブジェクトの座標付近に移動する

    Attributes
    ----------
    dist : float
        検出したオブジェクトまでの距離[m]
    obj_radius : float
        オブジェクトの半径[m]
    phi : float
        角度[rad]
    cam_frame : str
        カメラ座標系のフレーム名
    """

    def __init__(self):

        self.dist = 0.4
        self.obj_radius = 0.05
        self.phi = 0
        self.cam_frame = 'camera_link'

    def move_goal(self, point,
                  frame_time = 0):
        '''
        指定されたゴール地点に移動する
        Parameters
        ----------
        point : geometry_msgs/Point
            カメラ座標系のオブジェクト位置

        Returns
        -------
        client.get_result : Bool
            actionlibの実行結果
        '''
        # tfを取得
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        try:
            # カメラ → ベースリンクへの座標変換を取得
            trans1 = tfBuffer.lookup_transform('base_link', self.cam_frame, rospy.Time(0), rospy.Duration(3.0))
            # ベースリンク → マップへの座標変換を取得
            trans2 = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(3.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
        else:
            goal = self.transform_pose(self.calc_goal(self.transform_point(point, trans1)), trans2)
            goal_pose = MoveBaseGoal()
            goal_pose.target_pose.header.stamp = rospy.Time.now()
            goal_pose.target_pose.header.frame_id = 'map'
            goal_pose.target_pose.pose = goal

            client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            client.wait_for_server()
            #rospy.loginfo('connected to server')
            client.send_goal(goal_pose)
            rospy.loginfo('Send goal pos : [%5.3f, %5.3f, %5.3f]',
                          goal.position.x, goal.position.y, goal.position.z)
            self.set_marker(goal_pose)
            #client.wait_for_result()
            client.wait_for_result(rospy.Duration(10))
            rospy.loginfo(client.get_state())
			#http://docs.ros.org/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html
            Status = GoalStatus()
            if client.get_state() != Status.SUCCEEDED:
                client.cancel_goal()
                rospy.loginfo('Failed to Move')
            else:
                rospy.loginfo('Succeed to Move')
  
        return #client.get_state()

    def transform_point(self, point, trans):
        '''
        3次元の点(x,y,z)を座標変換する。

        Parameters
        ----------
        point : geometry_msgs/Point
            変換前の点
        trans : geometry_msgs/TransformStamped
            変換対象フレーム間のtransform

        Returns
        -------
        new_point : geometry_msgs/Point
            変換後の点
        '''
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
        '''
        3次元の姿勢((x,y,z), (x,y,z,w))を座標変換する。

        Parameters
        ----------
        point : geometry_msgs/Pose
            変換前の姿勢
        trans : geometry_msgs/TransformStamped
            変換対象フレーム間のtransform

        Returns
        -------
        new_pose : geometry_msgs/Pose
            変換後の姿勢
        '''

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
        '''
        オブジェクトの位置からゴール位置を計算する。

        Parameters
        ----------
        point : geometry_msgs/Point
            検出したオブジェクトの座標
        distance : float
            オブジェクトとゴール間の距離(m)
        object_radius : float
            オブジェクトの半径(m)
        phi : float
            ゴール姿勢の角度(rad)

        Returns
        -------
        pose : geometry_msgs/Pose
            ゴールの姿勢
        '''
        theta = np.arctan2(point.y, point.x)  #　現在地からオブジェクトまでの角度

        # ゴール位置の計算
        pose = Pose()
        pose.position.x = point.x + self.obj_radius*np.cos(theta) - (self.dist + self.obj_radius)*np.cos(theta - self.phi)
        pose.position.y = point.y + self.obj_radius*np.sin(theta) - (self.dist + self.obj_radius)*np.sin(theta - self.phi)
        pose.position.z = 0

        q = tf.transformations.quaternion_about_axis(theta - self.phi, (0,0,1))

        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        return pose


    def set_marker(self, goal_pose):
        """
        Rvizでゴール地点に矢印を表示する

        Parameters
        ----------
        goal_pose : geometry_msgs/Pose
            ゴールの姿勢

        """
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

        # topic : goal座標をRvizに配信
        pub_marker = rospy.Publisher('goal_point', Marker, queue_size=10)
        pub_marker.publish(marker_data)

if __name__ == '__main__':
    rospy.init_node('move_object_point')
    MC = MoveCoordinate()
    MC.cam_frame = 'base_link'
    point = Point()
    point.x = 0
    point.y = 0
    point.z = 0
    try:
        result = MC.move_goal(point)
    except rospy.ROSInterruptException: pass
