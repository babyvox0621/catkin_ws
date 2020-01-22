/*
 * 2017/02/16  Modify by Panasonic Corporation.
 */
#include "util/joy_node.h"

#define MAX_LIN_VEL 0.3
#define MAX_ANG_VEL 1.0

joy_handler::joy_handler()
{
  std::string  vel_topic_name;
  std::string  mode_topic_name;
  std::string  joy_topic_name;

  ros::NodeHandle pn("~");
  pn.param<std::string>("joy_topic",    joy_topic_name,     "joy");
  pn.param<std::string>("vel_topic",    vel_topic_name,     "cmd_vel");
  pn.param<std::string>("mode_topic",   mode_topic_name,    "mobile_base/command/mode");
  pn.param<std::string>("mode_cmd",     m_ModeCmd,          "normal");
  pn.param<int>("L1_BUTTON",            m_L1Btn,            4);
  pn.param<int>("R1_BUTTON",            m_R1Btn,            5);
  pn.param<int>("START_BUTTON",         m_StartBtn,         9);
  pn.param<int>("LEFT_Y_AXIS",          m_LYAxis,           0);
  pn.param<int>("LEFT_X_AXIS",          m_LXAxis,           1);
  pn.param<int>("RIGHT_Y_AXIS",         m_RYAxis,           2);
  pn.param<int>("RIGHT_X_AXIS",         m_RXAxis,           3);
  pn.param<int>("X_BUTTON",             m_XBtn,             4);
  pn.param<int>("Y_BUTTON",             m_YBtn,             5);

  joy_sub  = nodeh.subscribe<sensor_msgs::Joy>(joy_topic_name, 1, &joy_handler::joy_receive, this);
  twist_pub = nodeh.advertise<geometry_msgs::Twist>(vel_topic_name, 1);
  mode_pub = nodeh.advertise<std_msgs::String>(mode_topic_name, 1);

  twist.linear.x=0;
  twist.linear.y=0;
  twist.linear.z=0;
  twist.angular.x=0;
  twist.angular.y=0;
  twist.angular.z=0;
  vel_ = 1;
}

void joy_handler::joy_receive(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  static int zero_count=0;
  static int onFlag[3]={0,0,0};

  int angular_vel = vel_ > 3 ? 2 : vel_; 

  twist.linear.x = MAX_LIN_VEL*joy_msg->axes.at(m_LXAxis)*vel_;
  twist.angular.z = MAX_ANG_VEL*joy_msg->axes.at(m_LYAxis)*angular_vel;
  if ((twist.linear.x == 0)
      && (twist.angular.z == 0)) {
    if (zero_count < 100) {
      zero_count++;
    }
  } else {
    zero_count = 0;
  }
    
  if(joy_msg->buttons.at(m_StartBtn) && onFlag[0]==0) {
    mode.data=m_ModeCmd;
    ROS_INFO("SET %s MODE", m_ModeCmd.c_str());
    mode_pub.publish(mode);
    onFlag[0] = 1;
  } else if (!joy_msg->buttons.at(m_StartBtn)) {
    onFlag[0] = 0;
  }

  if(joy_msg->buttons.at(m_L1Btn) && onFlag[1]==0) {
    if (vel_<5) {
      vel_ += 1;
      ROS_INFO("Set vel_ %d", vel_);
    }
    onFlag[1] = 1;
  } else if (!joy_msg->buttons.at(m_L1Btn)) {
    onFlag[1] = 0;
  }
  if(joy_msg->buttons.at(m_R1Btn) && onFlag[2]==0) {
    if (vel_ > 1) {
      vel_ -= 1;
      ROS_INFO("Set vel_ %d", vel_);
    }
    onFlag[2] = 1;
  } else if (!joy_msg->buttons.at(m_R1Btn)) {
    onFlag[2] = 0;
  }
    
  if (zero_count<5) {
    twist_pub.publish(twist);
  }
}

joy_handler::~joy_handler()
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joy_node");

    joy_handler joy_h;

    ROS_INFO("Joy Handler Started");

    ros::spin();
}
