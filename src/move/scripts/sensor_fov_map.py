#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import Pose
import tf2_ros
import numpy as np
import tf

class precastDB:
	
	def __init__(self):
		self.d = 0.0
		self.angle = 0.0
		self.ix = 0
		self.iy = 0
		

class SensorFoVMap:
		
	def __init__(self):
		
		self.gridmap = OccupancyGrid()
		self.gridmap.info.width = 0
		self.gridmap.info.height = 0
		self.gridmap.data = []
		self.obstacle_map = OccupancyGrid()
		self.status = 0
							
		self.AoV = np.pi/2# Sensor Angle of View
		self.yawreso = np.deg2rad(10)# yaw resolution
		self.precast = self.precasting()
		
		self.sub_map = rospy.Subscriber('move_base/global_costmap/costmap', OccupancyGrid, self.callback_costmap)
		self.sub_map_updates = rospy.Subscriber('move_base/global_costmap/costmap_updates', OccupancyGridUpdate, self.callback_costmap_updates)
		
		r = rospy.Rate(10) #10Hz
		tfBuffer = tf2_ros.Buffer()
		listener = tf2_ros.TransformListener(tfBuffer)
		
		while not rospy.is_shutdown():
			try:
				trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(3.0))
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
				print(e)
			
			self.update_gridmap(trans)
			r.sleep()
			
	
	def callback_costmap(self, msg):
		
		self.obstacle_map = msg
		if self.gridmap.info != msg.info:
			
			if self.status == 0:
				self.gridmap.data = [ 0 for i in range(msg.info.width * msg.info.height)]		
				self.status = 1
				self.gridmap.info = msg.info
			else:
				self.cast_gridmap(msg)
				self.gridmap.info = msg.info
	
	def callback_costmap_updates(self, msg):
		
		data = np.array(self.obstacle_map.data)
		data = data.reshape([self.obstacle_map.info.height, self.obstacle_map.info.width])
		data[msg.y:msg.y+msg.height, msg.x:msg.x+msg.width] = np.array(msg.data).reshape([msg.height, msg.width])
		data = list(data.flatten())
		
	def update_gridmap(self, trans):
	
		if self.status == 1:
			reso = self.gridmap.info.resolution
			width = self.gridmap.info.width
			height = self.gridmap.info.height
			ox = self.gridmap.info.origin.position.x
			oy = self.gridmap.info.origin.position.y

			
			theta = 2 * np.arctan2(trans.transform.rotation.z, trans.transform.rotation.w)
			
			angle_min = int((theta - self.AoV/2) / self.yawreso + 0.5)
			angle_max = int((theta + self.AoV/2) / self.yawreso + 0.5)
			
			# (ix0, iy0) : current robot position (grid) 	
			ix0 = int((trans.transform.translation.x - ox) / reso + 0.5)
			iy0 = int((trans.transform.translation.y - oy) / reso + 0.5)
		
			obst_angle, obst_dist = self.calc_obstacle_pos(ix0, iy0, width, self.obstacle_map)
			
			# dmax : sensor measured distance
			dmax = 1 / reso
		
			for angle_id in range(angle_min, angle_max + 1):
				if angle_id < 0:
					angle_id += int(2 * np.pi / self.yawreso + 0.5)
				if angle_id >= int(2 * np.pi / self.yawreso + 0.5):
					angle_id -= int(2 * np.pi / self.yawreso + 0.5)
				idx = np.where(obst_angle == angle_id)[0]
				if len(idx) != 0:
					dist = np.min(obst_dist[idx])
					if dist > dmax:
						dist = dmax
				else:
					dist = dmax
				for grid in self.precast[angle_id]:
					if grid.d < dist :
						ix = grid.ix + ix0
						iy = grid.iy + iy0
						if (ix >= 0)&(iy >= 0)&(ix < width)&(iy < height):
							self.gridmap.data[ix + width*iy] = 100
								
			pub = rospy.Publisher('sensor_map', OccupancyGrid, queue_size=10)
			pub.publish(self.gridmap)
	
	def calc_obstacle_pos(self, ix0, iy0, width, obstacle_map):
	
		object_flag = 100 # LETHAL obstacle
		idx = np.where(np.array(obstacle_map.data) == object_flag)[0]
		pix = idx % width
		piy = idx // width

		obst_angle = np.arctan2((piy - iy0),(pix - ix0))
		obst_angle[np.where(obst_angle < 0)] = obst_angle[np.where(obst_angle < 0)] + np.pi*2
		obst_angle = (obst_angle / self.yawreso + 0.5).astype('int')

		obst_dist = np.sqrt((pix-ix0)**2 + (piy-iy0)**2)
		
		return obst_angle, obst_dist
							
	def precasting(self):

		imin = -int(2.0 / 0.025)
		imax = int(2.0 / 0.025)
		precast = [[] for i in range(int((np.pi * 2.0) / self.yawreso + 0.5))]
		
		for ix in range(imin, imax+1):
			for iy in range(imin, imax+1):
				angle = np.arctan2(iy, ix)
				
						
				d = np.sqrt(ix**2 + iy**2)
				
				pc = precastDB()
				pc.d = d
				pc.ix = ix
				pc.iy = iy
				pc.angle = angle
				
				angle_id = int(angle / self.yawreso + 0.5)
				if angle_id < 0:
					angle_id += int(2 * np.pi / self.yawreso + 0.5)
				
				precast[angle_id].append(pc)
				
		return precast

	def cast_gridmap(self, costmap):
		
		ox = self.gridmap.info.origin.position.x
		oy = self.gridmap.info.origin.position.y
		width = self.gridmap.info.width
		height = self.gridmap.info.height
		reso = self.gridmap.info.resolution
		
		new_ox = costmap.info.origin.position.x
		new_oy = costmap.info.origin.position.y
		new_width = costmap.info.width
		new_height = costmap.info.height
		
		newmap = np.zeros([new_height, new_width])
		oldmap = np.array(self.gridmap.data).reshape([height, width])
		
		trans_x = int((ox - new_ox) / reso + 0.5)
		trans_y = int((oy - new_oy) / reso + 0.5)
		
		print('trans', trans_x, trans_y)
		
		newmap[trans_y:trans_y+height, trans_x:trans_x+width] = oldmap
		newmap = list(newmap.flatten())
		self.gridmap.data = newmap
		
if __name__ == '__main__':
    rospy.init_node('sensorMap')
    try:
    	FoV = SensorFoVMap()
    	rospy.spin()
    except rospy.ROSInterruptException: pass
					
