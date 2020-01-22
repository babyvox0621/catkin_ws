#!/usr/bin/env python
import rospy
import actionlib
import move_goal
import adjust
import rondom_move
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

    def callback(self, msg):
        if((msg.node == 2)or(msg.node == 0)):
            send = commond()
            send.topic_id = msg.topic_id
            rospy.loginfo("%d", msg.topic_id)

            if(msg.msg.id == 0): #rondom
                rospy.loginfo("moving without goal")

                # simple version
                angle = np.deg2rad(5)
                liner = 0.2
                mov = rondom_move.Moving()
                mov.rotate(angle)
                self.cnt_angle += 1
                if self.cnt_angle == 12:
                    mov.liner_x(liner)
                    self.cnt_liner += 1
                    if self.cnt_liner == 3:
                        liner = -liner

                send.ret = 0 # succeed


            elif(msg.msg.id == 1): # move goal

                rospy.loginfo("moving with goal %f %f %f", msg.msg.x3d, msg.msg.y3d, msg.msg.z3d)
                point = Point()
                point.x = msg.msg.x3d
                point.y = msg.msg.y3d
                point.z = msg.msg.z3d
                MC = move_goal.MoveCoordinate()
                MC.cam_frame = 'camera_link'
                MC.move_goal(point)
                send.ret = 0 # succeed

            elif(msg.msg.id == 2): # adjust
                rospy.loginfo("position adjustment %d %d", msg.msg.x2d, msg.msg.y2d)

                point = Point()
                point.x = msg.msg.x3d
                point.y = msg.msg.y3d
                point.z = msg.msg.z3d
                cmd = adjust.Cmd()
                if(msg.msg.key == 0):
                    cmd.goal_z = 0.18
                elif(msg.msg.key ==1):
                    cmd.goal_z = 0.30
                cmd.rotate(point)
                cmd.back_and_forward(point)
                send.ret = 0 # succeed

            elif(msg.msg.id == 3): # move goal
                rospy.loginfo("moving with goal %f %f %f", 0,0,0)
                point = Point()
                point.x = 0
                point.y = 0
                point.z = 0
                MC = move_goal.MoveCoordinate()
                MC.cam_frame = 'map'
                MC.dist = 0
                MC.obj_radius = 0
                MC.move_goal(point)
                send.ret = 0 # succeed
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
