/**
 * Copyright(C) 2017 Panasonic Corporation. All Right reserved.
 * 
 * \file        pwm_keyboard_node.cpp
 * \brief       drive Rulo by PWM command.
 * \date        2017/02/16  create a new
 *
 * \note        
 *
 */

#include "util/pwm_keyboard_node.h"

int kfd = 0;
struct termios cooked, raw;

keyboard_handler::keyboard_handler()
{
    twist_pub = nodeh.advertise<rulo_msgs::DrivePwm>("drive_pwm", 1);
}

void keyboard_handler::keyboard_reading()
{
    char c;

    // get the console in raw mode                                                              
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file                         
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    ROS_INFO("Use arrow keys to move the robot.");

	pwm.left = 0;
	pwm.right = 0;

    while(1)
    {
	if(read(kfd, &c, 1) < 0)
	{
	    ROS_ERROR("Something wrong!");
	    exit(-1);
	}


	switch(c)
	{
	    case LEFT:
		pwm.right += 1;
		break;
	    case RIGHT:
		pwm.left += 1;
		break;
	    case FORWARD:
		pwm.left += 1;
		pwm.right += 1;
		break;
	    case BACKWARD:
		pwm.left -= 1;
		pwm.right -= 1;
		break;
	    case STOP:
		pwm.left = 0;
		pwm.right = 0;
	        break;
	}

	twist_pub.publish(pwm);    

	ros::spinOnce();

	usleep(100);
    }    
}

keyboard_handler::~keyboard_handler()
{

}

void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pwm_keyboard_node");

    keyboard_handler keyboard_h;

    ROS_INFO("Keyboard Handler Started");

    signal(SIGINT,quit);

    keyboard_h.keyboard_reading();

    return 0;
}
