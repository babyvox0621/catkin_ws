/**************************************************
 * copyright(c) panasonic.com
**************************************************/

#include "avoid_obstacle.h"
#include <sstream>
#include "std_msgs/String.h"
#include "tf/transform_broadcaster.h"

#define PI (3.14)

avoid_obstacle::avoid_obstacle()
{
    odom_sub = nodeh.subscribe<nav_msgs::Odometry>("/Rulo/odom", 10, &avoid_obstacle::odom_CB, this);
    scan_pub = nodeh.advertise<sensor_msgs::LaserScan>("/scan", 10);
}

void avoid_obstacle::odom_CB(const nav_msgs::Odometry::ConstPtr& _msg)
{
	// TODO: Not use Odometory. Use TF topic between "map" and "base_link".
    odom_ = *_msg;
}

void avoid_obstacle::scan_CB(const sensor_msgs::LaserScan::ConstPtr& _msg)
{
	// TODO:
	// This calculation costs the time.
    // If there is no virtual-wall, just pass-throw it.
      pub_odom(_msg);
}

void avoid_obstacle::pub_odom(const sensor_msgs::LaserScan::ConstPtr& _info)
{
  sensor_msgs::LaserScan laser_msg;
  laser_msg = *_info;

  // calculate object angle in LiDAR data.
  double one_data_angle_per_scan =
   (fabs(laser_msg.angle_min) + fabs(laser_msg.angle_max)) / laser_msg.ranges.size();

  //------- obstacle position-----
  // TODO: (1) Wrap these parameter to the message structure or class object.
  //           And get actual value from ROS topic or so.
  //       (2) Consider multiple virtual-wall. (ex) use vector and loop.

  // virtual-wall start point.
  double stX = 1.0;
  double stY = 5.0;

  // virtual-wall end point.
  double endX = 5.0;
  double endY = 1.0;


  //------- linear distance-----
  double stDiffX = odom_.pose.pose.position.x - stX;
  double stDiffY = odom_.pose.pose.position.y - stY;
  double stDiff  = sqrt(pow(stDiffX, 2) + pow(stDiffY, 2));

  double endDiffX = odom_.pose.pose.position.x - endX;
  double endDiffY = odom_.pose.pose.position.y - endY;
  double endDiff  = sqrt(pow(endDiffX, 2) + pow(endDiffY, 2));

  //------- wall direction -----
  // the direction to virtual-wall start point.
  double obstacleAngle1 = /*(PI/2)
         -*/
         atan2(
          stY - odom_.pose.pose.position.y  
        , stX - odom_.pose.pose.position.x  
         );
         /*+
         tf::getYaw(odom_.pose.pose.orientation);*/
	obstacleAngle1 -= tf::getYaw(odom_.pose.pose.orientation);

//printf("obstacleAngle1 [%f] \n", obstacleAngle1 );

  // the direction to virtual-wall end point.
  double obstacleAngle2 = 
         atan2(
         endY - odom_.pose.pose.position.y  
        , endX - odom_.pose.pose.position.x  
         );
	obstacleAngle2 -= tf::getYaw(odom_.pose.pose.orientation);
//printf("obstacleAngle2 [%f] \n", obstacleAngle2 );
 
  double object_left_angle;
  double object_right_angle;
  double linearDiffPerAngle;
  double distDiff;
  if (obstacleAngle1 < obstacleAngle2)
  {
		object_left_angle = obstacleAngle1;
		object_right_angle = obstacleAngle2;
  		linearDiffPerAngle = (endDiff - stDiff) / fabs(object_right_angle - object_left_angle);
		distDiff = stDiff;
  } else {
		object_left_angle = obstacleAngle2;
		object_right_angle = obstacleAngle1;
  		linearDiffPerAngle = (stDiff - endDiff) / fabs(object_left_angle - object_right_angle);
		distDiff = endDiff;
  }
  linearDiffPerAngle = linearDiffPerAngle * one_data_angle_per_scan;


ROS_INFO("object_left_angle[%f]", object_left_angle);
ROS_INFO("object_right_angle[%f]", object_right_angle);
ROS_INFO("stDiff[%f]", stDiff);
ROS_INFO("endDiff[%f]", endDiff);
ROS_INFO("linearDiffPerAngle[%f]", linearDiffPerAngle);
ROS_INFO("one_data_angle_per_scan[%f]", one_data_angle_per_scan);

  // overwrite scan renges.
  auto rangeItr = laser_msg.ranges.begin();
  //auto intensityItr = laser_msg.intensities.begin();
  double curent_angle = laser_msg.angle_min;

  while (rangeItr != laser_msg.ranges.end())
  {
     if (curent_angle > object_left_angle
      && curent_angle < object_right_angle)
     {
         *rangeItr = distDiff;   
		 distDiff += linearDiffPerAngle;
         //*intensityItr = 255;
     } else {
     }
     rangeItr++;
     //intensityItr++;
     curent_angle += one_data_angle_per_scan;
  }
  scan_pub.publish(laser_msg);
}




avoid_obstacle::~avoid_obstacle()
{

}

void avoid_obstacle::setPrefix(std::string _info) {
    if ( _info != "" ) {
        this->prefix_sub_ = "/" + _info;
    }
    scan_sub = nodeh.subscribe<sensor_msgs::LaserScan>(this->prefix_sub_ + "scan", 10, &avoid_obstacle::scan_CB, this);

    ROS_INFO("subscribe prefix = %s", _info.c_str());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "avoid_obstacle");

    avoid_obstacle ao_h;

    ao_h.setPrefix("Rulo/");

    ROS_INFO("Avoid Obstacle Started");

    ros::spin();
}

