#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib
import move_goal
import adjust
#import rondom_move
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PointStamped, Point, Pose
from move.msg import commond, MSG
from visualization_msgs.msg import Marker
import tf2_ros
import tf
import numpy as np

class Rulo:

    def __init__(self):

        self.sub_rulo = rospy.Subscriber('Commond_TOP', commond, self.callback)
        self.pub_rulo = rospy.Publisher('results_RULO', commond, queue_size=1)
        self.cnt_angle = 0
        self.cnt_liner = 0

        # get current position 
        MC = move_goal.MoveCoordinate()
        point = Point()
        self.point_map_0 = MC.transform_point(point, MC.get_tf('map', 'base_link'))

    def callback(self, msg):
        if((msg.node == 2)or(msg.node == 0)):
            send = commond()
            send.topic_id = msg.topic_id
            rospy.loginfo("%d", msg.topic_id)

            if(msg.msg.id == 0): # random move
                rospy.loginfo("moving without goal")

                # simple version
                Cmd = adjust.Cmd()
                MC = move_goal.MoveCoordinate()
                rot_amount = np.deg2rad(45) # rotation amount of random move
                move_forward_amount = 0.3
                Cmd.rotate(rot_amount)
                self.cnt_angle += 1
                if self.cnt_angle*rot_amount == 2*np.pi:

                    #ランダムな方向設定
                    '''
                    angle_list = [0,45,90,135,180,225,270,315]
                    old_direct = 0
                    ret = 1

                    while((len(angle_list)!=0)or(ret!=0)):
                        direction = angle_list[np.random.randint(0,len(angle_list))]
                        Cmd.rotate(np.deg2rad(direction))
                        old_direct += direction
                        if old_direct > 360:
                            old_direct = old_direct - 360

                        point_base = Point()
                        point_base.x = move_forward_amount
                        MC.dist = 0
                        ret = MC.move_pose(MC.calcgoal(Point_base))
                    '''
                    point_base = Point()
                    point_base.x = move_forward_amount
                    MC.dist = 0
                    ret = MC.move_pose(MC.calcgoal(Point_base))
                    self.cnt_angle = 0

                send.ret = 0 # succeed


            elif(msg.msg.id == 1): # move target

                rospy.loginfo("moving with goal %f %f %f", msg.msg.x3d, msg.msg.y3d, msg.msg.z3d)
                point_cam = Point()
                point_cam.x = msg.msg.x3d
                point_cam.y = msg.msg.y3d
                point_cam.z = msg.msg.z3d

                MC = move_goal.MoveCoordinate()
                MC.move_obj(point_cam)
                send.ret = 0 # succeed

            elif(msg.msg.id == 2): # adjust
                rospy.loginfo("position adjustment %d %d", msg.msg.x2d, msg.msg.y2d)

                point_cam = Point()
                point_cam.x = msg.msg.x3d
                point_cam.y = msg.msg.y3d
                point_cam.z = msg.msg.z3d

                Cmd = adjust.Moving()
                MC = move_goal.MoveCoordinate()

                #座標変換
                point_base = MC.transform_point(point_cam, MC.trans_cam2base)
                angle = np.arctan2(point_base.x, point_base.y)
                Cmd.rotate(angle) # 回転
                if(msg.msg.key == 0):
                    distance = np.sqrt(point_base.x**2 + point_base.y**2) - 0.18 # オブジェクトへの移動
                elif(msg.msg.key ==1):
                    distance = np.sqrt(point_base.x**2 + point_base.y**2) - 0.30 # 人への移動
                Cmd.back_and_forward(distance) # 直進
                send.ret = 0 # succeed

            elif(msg.msg.id == 3): # move origin
                rospy.loginfo("moving with goal %f %f %f", 0,0,0)
                point_map = self.point_map_0
                #point_map.x = 0
                #point_map.y = 0
                #point_map.z = 0
                MC = move_goal.MoveCoordinate()
                Cmd = adjust.Cmd()
                trans_map2base = MC.get_tf('base_link', 'map')
                point_base = MC.transform_point(point_map, trans_map2base)
                Cmd.rotate(point_base) # 移動方向に回転
                MC.move_pose(point_map) # 自律移動
                send.ret = 0 # succeed

            elif(msg.msg.id == 4): # turn towards object
                
                point_cam = Point()
                point_cam.x = msg.msg.x3d
                point_cam.y = msg.msg.y3d
                point_cam.z = msg.msg.z3d

                Cmd = adjust.Moving()
                MC = move_goal.MoveCoordinate()

                #座標変換
                point_base = MC.transform_point(point_cam, MC.trans_cam2base)
                angle = np.arctan2(point_base.x, point_base.y)
                rospy.loginfo("turn %f degree", np.rad2deg(angle))
                Cmd.rotate(angle) # 回転

            else: # invalid move mode
                rospy.loginfo("invalid move mode @RULO")

            self.pub_rulo.publish(send)
        else:
            rospy.loginfo("this topic is not for rulo")

if __name__ == '__main__':
    rospy.init_node('rulo')
    try:
        RULO = Rulo()
        rospy.loginfo("running...")
        rospy.spin()
    except rospy.ROSInterruptException: pass
    


