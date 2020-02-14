#!/usr/bin/env python

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from visualization_msgs.msg import Marker
import tf2_ros
import numpy as np

class Explorer:
    
    def __init__(self):

        self.sub_costmap = rospy.Subscriber('move_base/global_costmap/costmap', OccupancyGrid, self.callback_costmap)
        self.sub_costmap_updates = rospy.Subscriber('move_base/global_costmap/costmap_updates', OccupancyGridUpdate, self.callback_costmap_updates)
        self.sub_sensormap = rospy.Subscriber('sensorMap', OccupancyGrid, self.callback_sensormap)
        self.pub = rospy.Publisher('arrow_pub', Marker, queue_size=10)
        self.status = 0
        self.goal = [0,0]
        
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

		r = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            if self.status != 0:
                try:
                    trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(3.0))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    print(e)
                self.update_distmap(trans)            
                goal_pose = self.get_goal()
                self.pub.publish(self.set_marker(goal_pose))
                client.send_goal(goal_pose)
                client.wait_for_result()
                r.sleep()

    def callback_costmap(self, msg):
		
        self.costmap = msg
        self.width = msg.info.width
        self.height = msg.info.height
        self.reso = msg.info.resolution
        self.ox = msg.info.origin.position.x
        self.oy = msg.info.origin.position.y
        self.status = 1
	
    
    def callback_costmap_updates(self, msg):
        
        data = np.array(self.costmap.data)
        data = data.reshape([self.height, self.width])
        data[msg.y:msg.y+msg.height, msg.x:msg.x+msg.width] = np.array(msg.data).reshape([msg.height, msg.width])
        data = data.flatten().tolist()


    def callback_sensormap(self, msg):
        
        self.sensormap = msg


    def update_distmap(self,trans):
        
        cx = int((trans.transform.translation.x - self.ox) / self.reso + 0.5)
        cy = int((trans.transform.translation.y - self.oy) / self.reso + 0.5)
        
        X, Y = np.meshgrid(range(-cx, self.width - cx), range(-cy, self.height - cy))
        self.distmap = np.sqrt(X**2 + Y**2).flatten()


    def get_goal(self):
        
        try:
            cand = np.where((np.array(self.costmap.data) == 0)&(np.array(self.sensormap) == 0))[0]
            target_idx = cand[np.argmin(self.distmap[cand])]
            self.goal = [target_idx % self.width * self.reso + self.ox, target_idx // self.width * self.reso + self.oy]
        except Exception as e:
            print(e)
        
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = "map"
        goal_pose.target_pose.header.stamp = rospy.Time.now()
        goal_pose.target_pose.pose.position.x = self.goal[0]
        goal_pose.target_pose.pose.position.y = self.goal[1]
        goal_pose.target_pose.pose.position.z = 0
        goal_pose.target_pose.pose.orientation.x = 0
        goal_pose.target_pose.pose.orientation.y = 0
        goal_pose.target_pose.pose.orientation.z = 0
        goal_pose.target_pose.pose.orientation.w = 1
    
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
    
    rospy.init_node('sensorMap')
    try:
        Exp = Explorer()
        rospy.spin()
    except rospy.ROSInterruptException: 
        pass