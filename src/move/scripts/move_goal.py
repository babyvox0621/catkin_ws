#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker
from actionlib_msgs.msg import GoalStatus
import tf2_ros
import tf
import numpy as np


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

        self.cam_frame = 'camera_link'
        self.base_frame = 'base_link'
        self.map_frame = 'map'
        # self.trans_cam2base = self.get_tf(self.base_frame, self.cam_frame)
        # self.dist = 0.40

    """
    def move_obj(self, point_cam):

        # 座標変換
        trans_base2map = self.get_tf(self.map_frame, self.base_frame)
        point_base = self.transform_point(point_cam, self.trans_cam2base)
        print("point_base", point_base)
        pose_base = self.calc_goal(point_base, )
        print("pose_base", pose_base)
        pose_map = self.transform_pose(pose_base, trans_base2map)
        print("pose_map", pose_map)
        # 移動
        ret = self.move_pose(pose_map)
        return ret
    """

    def move_pose(self, pose_map):
        '''
        指定されたゴール姿勢に移動する
        Parameters
        ----------
        pose_map : geometry_msgs/Pose
            map座標系のポーズ

        Returns
        -------
        ret: int
            actionlibの実行結果 0:succeed 1:failed
        '''

        goal = pose_map
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.stamp = rospy.Time.now()
        goal_pose.target_pose.header.frame_id = self.map_frame
        goal_pose.target_pose.pose = goal

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        # rospy.loginfo('connected to server')
        client.send_goal(goal_pose)
        rospy.loginfo('Send goal pos : [%5.3f, %5.3f, %5.3f]',
                      goal.position.x, goal.position.y, goal.position.z)
        self.set_marker(goal_pose)
        client.wait_for_result(rospy.Duration(10))
        # http://docs.ros.org/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html
        Status = GoalStatus()
        while(client.get_state() != Status.SUCCEEDED):
            if((client.get_state() != Status.ACTIVE)or
                    (client.get_state() != Status.SUCCEEDED)):
                client.wait_for_result(rospy.Duration(5))  # timeout時間を設定
                if(client.get_state() != Status.ACTIVE):
                    break
            client.wait_for_result(rospy.Duration(1))

        if(client.get_state() == Status.SUCCEEDED):
            rospy.loginfo('Succeed to Move')
            ret = 0
        else:
            client.cancel_goal()
            rospy.loginfo('Failed to Move')
            ret = 1

        return ret

    def get_tf(self, after_frame, before_frame):
        '''
        tfを取得する

        Parameters
        ----------
        after_frame : strings
            変換後のフレーム
        before_frame : strings
            変換前のフレーム

        Returns
        -------
        trans : tf
            tf
        '''
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        try:
            trans = tfBuffer.lookup_transform(after_frame,
                                              before_frame,
                                              rospy.Time(0),
                                              rospy.Duration(3.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            print(e)
        else:
            return trans

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
        pose : geometry_msgs/Pose
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

        new_ornt = tf.transformations.quaternion_multiply(q, ornt)
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

    def calc_goal(self, point_base, dist=0.4):  # , obj_radius=0, phi=0):
        '''
        オブジェクトの位置からゴール位置を計算する。

        Parameters
        ----------
        point_base : geometry_msgs/Point
            検出したオブジェクトの座標(base_link座標系)
        obj_radius : float, optional
            オブジェクトの半径(m)
        phi : float
            ゴール姿勢の角度(rad), optional

        Returns
        -------
        pose : geometry_msgs/Pose
            ゴールの姿勢
        '''
        theta = np.arctan2(point_base.x, point_base.y)  # 現在地からオブジェクトまでの角度

        # ゴール位置の計算
        pose = Pose()
        pose.position.x = point_base.x - dist * np.sin(theta)
        pose.position.y = point_base.y - dist * np.cos(theta)
        pose.position.z = 0

        q = tf.transformations.quaternion_about_axis(theta - np.pi, (0, 0, 1))

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
        marker_data.header.frame_id = self.map_frame
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
    point = Point()
    point.x = 0
    point.y = 0
    point.z = 0
    try:
        result = MC.move_goal(point)
    except rospy.ROSInterruptException:
        pass
