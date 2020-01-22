/**
 * Copyright(C) 2017 Panasonic Corporation. All Right reserved.
 * 
 * \file        pwm_keyboard_node.h
 * \brief       drive Rulo by PWM command.
 * \date        2017/02/16  create a new
 *
 * \note        
 *
 */

#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define RIGHT 0x43 
#define LEFT 0x44
#define FORWARD 0x41
#define BACKWARD 0x42
#define STOP 0x20

#include "ccr_msgs/DrivePwm.h"

class keyboard_handler
{
public:
  keyboard_handler();
  void keyboard_reading();
  ~keyboard_handler();
private:

  ros::NodeHandle nodeh;
  ros::Publisher twist_pub;
  ccr_msgs::DrivePwm pwm;
};
