/**************************************************
 * copyright(c) panasonic.com
**************************************************/

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include "sensor_msgs/LaserScan.h"
#include <nav_msgs/Odometry.h>

// Camera sight radius (radian)
#define MAX_CAMERA_RAD (0.785)
#define MIN_CAMERA_RAD (-0.785)
// obstacle range(m)
#define OBSTACLE_RANGE (2.0)
// obstacle recognition time.
#define OBSTACLE_RECOG_TIME (5.0)

class avoid_obstacle
{
public:
  avoid_obstacle();
  ~avoid_obstacle();

  void setPrefix(std::string _info);
private:
  ros::NodeHandle nodeh;
  ros::Subscriber scan_sub, odom_sub, command_sub;
  ros::Publisher scan_pub;

  nav_msgs::Odometry odom_;
  nav_msgs::Odometry odom_when_found_obstacle_;
  ros::Time prev_time_;
  bool isAvoiding_;
  double cur_range_;
  int obs_cnt_;
  int obs_left_cnt_;
  int obs_right_cnt_;

  void scan_CB(const sensor_msgs::LaserScan::ConstPtr& _msg);
  void odom_CB(const nav_msgs::Odometry::ConstPtr& _msg);

  void pub_odom(const sensor_msgs::LaserScan::ConstPtr& _msg);

  std::string prefix_sub_;
  std::string frame_name_;
};
