#!/usr/bin/env python

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from move.msg import coordinate
from visualization_msgs.msg import Marker
import tf2_ros
import numpy as np

class Explorer:

    def __init__(self):

        self.costmap = OccupancyGrid()
        self.sensormap = OccupancyGrid()
        DIST_CONST = 1
        self.is_detected = False
        self.dist_min = DIST_CONST
        self.dist = DIST_CONST
        self.goal = [0,0]


        self.sub_costmap = rospy.Subscriber('move_base/global_costmap/costmap', OccupancyGrid, self.callback_costmap)
        self.sub_costmap_updates = rospy.Subscriber('move_base/global_costmap/costmap_updates', OccupancyGridUpdate, self.callback_costmap_updates)
        self.sub_sensormap = rospy.Subscriber('sensor_map', OccupancyGrid, self.callback_sensormap)
        self.sub_pose_info = rospy.Subscriber('pose_info', coordinate, self.callback_pose_info)
        self.pub = rospy.Publisher('arrow_pub', Marker, queue_size=10)
        

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #rospy.Timer(rospy.Duration(0.5), self.timercallback)
		#r = rospy.Rate(0.5)

        while not rospy.is_shutdown():
            if self.is_detected:
                break
            try:
                trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(3.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(e)

            else:
            	self.update_distmap(trans)
            
            try:
                goal_pose = self.get_goal()
            except Exception as e:
				print(e)
            #print(goal_pose)
            else:
                #print(goal_pose)
                self.pub.publish(self.set_marker(goal_pose))
                client.send_goal(goal_pose)
                print('send goals')
                print(client.get_state())
                client.wait_for_server(rospy.Duration(10))
                if client.get_state == '0':
                    client.cancel_goal()
                    print('cancel goal')
                    self.dist_min = self.dist
                else:
                    isDone = client.wait_for_result(rospy.Duration(10))
                    if isDone == True :
                        print('succeed')
                        self.dist_min = DIST_CONST
                    else:
                        print('fali to reach goal')
                        self.dist_min = self.dist
                # 1:This goal has been accepted by the simple action server

                #while not (rospy.is_shutdown())or(cnt > 50):
                #client.get_state()
                #print(client.get_goal_status_text())
                #print(client.get_state())
                #rospy.Rate(2).sleep()
                #cnt+=1
                #self.client.wait_for_result()
    
    #def timercallback(self, event):
     #   print(self.client.get_state())
     #   print(self.client.get_goal_status_text())
    
    def callback_costmap(self, msg):

        self.costmap = msg
        self.width = msg.info.width
        self.height = msg.info.height
        self.reso = msg.info.resolution
        self.ox = msg.info.origin.position.x
        self.oy = msg.info.origin.position.y


    def callback_costmap_updates(self, msg):

        data = np.array(self.costmap.data)
        data = data.reshape([self.height, self.width])
        data[msg.y:msg.y+msg.height, msg.x:msg.x+msg.width] = np.array(msg.data).reshape([msg.height, msg.width])
        data = data.flatten().tolist()


    def callback_sensormap(self, msg):

        self.sensormap = msg

    def callback_pose_info(self, msg):

        self.is_detected = msg.detected

    def update_distmap(self,trans):

        self.cx = int((trans.transform.translation.x - self.ox) / self.reso + 0.5)
        self.cy = int((trans.transform.translation.y - self.oy) / self.reso + 0.5)

        X, Y = np.meshgrid(range(-self.cx, self.width - self.cx), range(-self.cy, self.height - self.cy))
        self.distmap = np.sqrt(X**2 + Y**2).flatten()


    def get_goal(self):

        try:
            #print(len(np.array(self.costmap.data) == 0), len(np.array(self.sensormap.data) == 0))
            cand = np.where((np.array(self.costmap.data) == 0)&
                            (np.array(self.sensormap.data) == 0)&
                            (self.distmap.data > self.dist_min))
            #print(len(cand[0]))
            target_idx = cand[0][np.argmin(self.distmap[cand])]
            print(target_idx)
            self.dist = self.distmap[target_idx]
            self.goal = [target_idx % self.width * self.reso + self.ox, target_idx // self.width * self.reso + self.oy]
            self.angle = np.arctan2(target_idx // self.width - self.cy, target_idx % self.width - self.cx)
        except Exception as e:
            print(e)
        #print(np.deg2rad(angle))
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = "map"
        goal_pose.target_pose.header.stamp = rospy.Time.now()
        goal_pose.target_pose.pose.position.x = self.goal[0]
        goal_pose.target_pose.pose.position.y = self.goal[1]
        goal_pose.target_pose.pose.position.z = 0
        goal_pose.target_pose.pose.orientation.x = 0
        goal_pose.target_pose.pose.orientation.y = 0
        goal_pose.target_pose.pose.orientation.z = np.sin(self.angle/2)
        goal_pose.target_pose.pose.orientation.w = np.cos(self.angle/2)

        return goal_pose



    def set_marker(self, goal_pose):

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

        return marker_data


if __name__ == '__main__':

    rospy.init_node('explorer')
    try:
        Exp = Explorer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
