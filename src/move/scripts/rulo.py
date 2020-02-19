#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import move_goal
import adjust
from geometry_msgs.msg import Point
from move.msg import commond
import numpy as np


class Rulo:

    def __init__(self):

        self.sub_rulo = rospy.Subscriber('Commond_TOP', commond, self.callback)
        self.pub_rulo = rospy.Publisher('results_RULO', commond, queue_size=1)
        self.cnt_angle = 0

        # get current position
        MC = move_goal.MoveCoordinate()
        point = Point()
        self.point_map_0 = MC.transform_point(point,
                                              MC.get_tf('map', 'base_link'))

    def callback(self, msg):
        if((msg.node == 2)or(msg.node == 0)):
            send = commond()
            send.topic_id = msg.topic_id
            rospy.loginfo("%d", msg.topic_id)

            if(msg.msg.id == 0):  # random move
                rospy.loginfo("Move without goal")
                Adj = adjust.Adjust()
                Adj.angular_speed = np.deg2rad(20)
                Adj.rotate(np.deg2rad(20))
                #print("ret:", ret)
                self.cnt_angle += 1
                """
                if(self.cnt_angle * np.deg2rad(30) == 2 * np.pi):
                    self.cnt_angle = 0
                    angle = np.random.rand() * 2 * np.pi
                    if(angle > np.pi):
                        angle = angle - 2 * np.pi
                    Adj.rotate(angle)

                    MC = move_goal.MoveCoordinate()
                    point_base = Point()
                    point_base.x = 0.3
                    point_base.y = 0
                    point_base.z = 0
                    pose_base = MC.calc_goal(point_base, dist=0)
                    pose_map = MC.transform_pose(pose_base,
                                                MC.get_tf('map', 'base_link'))
                    ret = MC.move_pose(pose_map)

                    print("goal@base", pose_base)
                    print("-------------------------------------------")
                    print("goal@map", pose_map)
                    print("-------------------------------------------")
                """
                send.ret = 0  # succeed

            elif(msg.msg.id == 1):  # move target

                rospy.loginfo("Move with goal %f %f %f", msg.msg.x3d, msg.msg.y3d, msg.msg.z3d)

                self.cnt_angle = 0
                point_cam = Point()
                point_cam.x = msg.msg.x3d
                point_cam.y = msg.msg.y3d
                point_cam.z = msg.msg.z3d

                MC = move_goal.MoveCoordinate()
                point_base = MC.transform_point(point_cam,
                                               MC.get_tf('base_link', 'camera_link'))
                pose_base = MC.calc_goal(point_base, dist=0.6)
                pose_map = MC.transform_pose(pose_base,
                                             MC.get_tf('map', 'base_link'))
                MC.move_pose(pose_map)

                print("target@cam", point_cam)
                print("-------------------------------------------")
                print("target@base", point_base)
                print("-------------------------------------------")
                print("goal@base", pose_base)
                print("-------------------------------------------")
                print("goal@map", pose_map)
                print("-------------------------------------------")

                send.ret = 0  # succeed

            elif(msg.msg.id == 2):  # adjust

                rospy.loginfo("Adjust position")

                point_cam = Point()
                point_cam.x = msg.msg.x3d
                point_cam.y = msg.msg.y3d
                point_cam.z = msg.msg.z3d

                MC = move_goal.MoveCoordinate()
                point_base = MC.transform_point(point_cam,
                                                MC.get_tf('base_link', 'camera_link'))
                theta = np.arctan2(point_base.y, point_base.x)

                Adj = adjust.Adjust()
                Adj.angular_speed = np.deg2rad(20)
                Adj.rotate(theta)  # 回転

                if(msg.msg.key == 0):  # オブジェクトへの移動
                    distance = np.sqrt(point_base.x**2 + point_base.y**2) - 0.405
                elif(msg.msg.key == 1):  # 人への移動
                    distance = np.sqrt(point_base.x**2 + point_base.y**2) - 0.405
                Adj.back_and_forward(distance)  # 直進

                print("target@cam", point_cam)
                print("-------------------------------------------")
                print("theta :%f", np.rad2deg(theta))
                print("-------------------------------------------")
                print("distance :%f", distance)
                print("-------------------------------------------")
                send.ret = 0  # succeed

            elif(msg.msg.id == 3):  # move origin
                rospy.loginfo("Move origin")
                MC = move_goal.MoveCoordinate()
                # point_map.x = 0
                # point_map.y = 0
                # point_map.z = 0
                point_map = self.point_map_0
                point_base = MC.transform_point(point_map,
                                                MC.get_tf('base_link', 'map'))
                pose_base = MC.calc_goal(point_base, dist=0)
                pose_map = MC.transform_pose(pose_base,
                                             MC.get_tf('map', 'base_link'))

                theta = np.arctan2(point_base.y, point_base.x)
                Adj = adjust.Adjust()
                Adj.angular_speed = np.deg2rad(45)
                Adj.rotate(theta)  # 移動方向に回転
                MC.move_pose(pose_map)  # 自律移動

                print("goal@base ", pose_base)
                print("-------------------------------------------")
                print("goal@map ", pose_map)
                print("-------------------------------------------")
                print("theta :%f", np.rad2deg(theta))
                print("-------------------------------------------")

                send.ret = 0  # succeed

            elif(msg.msg.id == 4):  # turn towards object
                rospy.loginfo("Turn towards target")
                point_cam = Point()
                point_cam.x = msg.msg.x3d
                point_cam.y = msg.msg.y3d
                point_cam.z = msg.msg.z3d

                Adj = adjust.Adjust()
                MC = move_goal.MoveCoordinate()

                point_base = MC.transform_point(point_cam, MC.trans_cam2base)
                theta = np.arctan2(point_base.y, point_base.x)
                rospy.loginfo("turn %f degree", np.rad2deg(theta))
                Adj.rotate(theta)  # 回転

                print("theta :%f", np.rad2deg(theta))
                print("-------------------------------------------")

                send.ret = 0

            else:  # invalid move mode
                rospy.loginfo("invalid move mode @RULO")

            self.pub_rulo.publish(send)
            rospy.loginfo('published')
        else:
            rospy.loginfo("this topic is not for rulo")

if __name__ == '__main__':
    rospy.init_node('rulo')
    try:
        RULO = Rulo()
        rospy.loginfo("running...")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
