/**
 * Copyright(C) 2017 Panasonic Corporation. All Right reserved.
 * 
 * \file        Ccr.cpp
 * \brief       co-creation robot base controller.
 * \date        2017/02/16  create a new
 *
 * \note        [説明]
 *
 */
#define NODE_VERSION 2.01

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>				// odom
#include <std_msgs/Bool.h>
#include <std_msgs/Byte.h>
#include <ccr_msgs/Battery.h>		// battery
#include <ccr_msgs/Bumper.h>		// bumper
#include <ccr_msgs/IRBumper.h>		// ir_bumper cliff
#include <ccr_msgs/WheelDrop.h>	// wheel_drop
#include <sensor_msgs/Range.h>
#include <ccr_msgs/Slip.h>
#include <ccr_msgs/Drive.h>
#include <ccr_msgs/BooleanSensor.h>
#include <ccr_msgs/Brushes.h>
#include <ccr_msgs/DirtDetect.h>
#include <ccr_msgs/IRCharacter.h>
#include <ccr_msgs/LEDBoardEvent.h>
#include <sensor_msgs/LaserScan.h>

#include "SerialDataPicker.h"
#include "diversion/odometry.h"
#include "Ccr_Constants.h"
#include "Ccr.h"

#include <string>
#include <atomic>
#include <eigen3/Eigen/Eigen>
#include <XmlRpc.h>
#include <algorithm>

//--->for atomic serial access
//#define __ATOMIC_ACCESS__
#ifdef __ATOMIC_ACCESS__
#include <mutex>
std::mutex g_guard;
#endif
//<---for atomic serial access

#define VEL_THRESHOLD (1.0)
#define CALIB_THRESHOLD (50)
#define MIN_ROTATION (0.1)
// *****************************************************************************
// Constructor.
ccr::Ccr::Ccr()
{
}

// *****************************************************************************
// Destructor.
ccr::Ccr::~Ccr()
{
	mobile_base->tearDown();
}

// *****************************************************************************
// Initialization.
void ccr::Ccr::init()
{
    rosnode_ = boost::shared_ptr<ros::NodeHandle> ( new ros::NodeHandle () );

	// ******************************************************************************************
	// get and set launch parameters.
	ros::NodeHandle pn("~");

	std::string port;
	//pn.param<std::string>("port_", port, "/dev/ttyACM0");
pn.param<std::string>("port_", port, "/dev/ttyTHS2");

	pn.param<std::string>("base_frame_id", base_frame_id, "/base_link");
	pn.param<std::string>("odom_frame_id", odom_frame_id, "/odom");
    
    pn.param<bool>("publishTf", publishTf, true);
    pn.param<bool>("use_imu", use_imu_, false);

	pose_cov_mat = false;
	twist_cov_mat = false;
    Eigen::MatrixXd poseCovariance(6,6);
    Eigen::MatrixXd twistCovariance(6,6);
    XmlRpc::XmlRpcValue poseCovarConfig;
    XmlRpc::XmlRpcValue twistCovarConfig;
    
    if (pn.hasParam("poseCovariance"))
    {
        try
        {
            pn.getParam("poseCovariance", poseCovarConfig);

            ROS_ASSERT(poseCovarConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

            int matSize = poseCovariance.rows();

            if (poseCovarConfig.size() != matSize * matSize)
            {
                ROS_WARN_STREAM("Pose_covariance matrix should have " << matSize * matSize << " values.");
            }
            else
            {
                for (int i = 0; i < matSize; i++)
                {
                    for (int j = 0; j < matSize; j++)
                    {
                        std::ostringstream ostr;
                        ostr << poseCovarConfig[matSize * i + j];
                        std::istringstream istr(ostr.str());
                        istr >> poseCovariance(i, j);
                        pose_covariance_matrix.push_back(poseCovariance(i,j));
                    }
                }
                pose_cov_mat = true;
            }
        }
        catch (XmlRpc::XmlRpcException &e)
        {
            ROS_ERROR_STREAM("ERROR reading sensor config: " << e.getMessage() << " for pose_covariance (type: " << poseCovarConfig.getType() << ")");
        }
        
    }
    
    if (pn.hasParam("twistCovariance"))
    {
        try
        {
            pn.getParam("twistCovariance", twistCovarConfig);

            ROS_ASSERT(twistCovarConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

            int matSize = twistCovariance.rows();

            if (twistCovarConfig.size() != matSize * matSize)
            {
                ROS_WARN_STREAM("Twist_covariance matrix should have " << matSize * matSize << " values.");
            }
            else
            {
                for (int i = 0; i < matSize; i++)
                {
                    for (int j = 0; j < matSize; j++)
                    {
                        std::ostringstream ostr;
                        ostr << twistCovarConfig[matSize * i + j];
                        std::istringstream istr(ostr.str());
                        istr >> twistCovariance(i, j);
                        twist_covariance_matrix.push_back(twistCovariance(i,j));
                    }
                }
                twist_cov_mat = true;
            }
        }
        catch (XmlRpc::XmlRpcException &e)
        {
            ROS_ERROR_STREAM("ERROR reading sensor config: " << e.getMessage() << " for twist_covariance (type: " << poseCovarConfig.getType() << ")");
        }
    }

	// ******************************************************************************************
	// setup for publish/subscribe topics. 

    // get settings which topic is published.
    pn.param<bool>("has_cleaning_module", has_cleaning_module_, false);
    pn.param<bool>("has_led_board", has_led_board_, false);
    pn.param<bool>("has_ir_receiver", has_ir_receiver_, false);
    pn.param<bool>("has_opt_left", has_opt_left_, false);
    pn.param<bool>("has_opt_right", has_opt_right_, false);
    pn.param<bool>("has_us_left", has_us_left_, false);
    pn.param<bool>("has_us_right", has_us_right_, false);
    pn.param<bool>("has_us_front", has_us_front_, false);
    pn.param<bool>("has_us_back", has_us_back_, false);
    pn.param<bool>("has_slip_sensor", has_slip_sensor_, false);
    pn.param<bool>("has_cliff", has_cliff_, false);
    pn.param<bool>("has_wheel_drop", has_wheel_drop_, false);
    pn.param<bool>("has_bumper", has_bumper_, false);
    pn.param<bool>("has_battery_config", has_battery_config_, false);

	odom_pub = rosnode_->advertise<nav_msgs::Odometry>("odom", 50);
    if (has_battery_config_)
    {
        battery_pub = rosnode_->advertise<ccr_msgs::Battery>(TOPIC_EVENT_PREFIX"battery", 50);
    }
    if (has_bumper_)
    {
        bumper_pub = rosnode_->advertise<ccr_msgs::Bumper>(TOPIC_EVENT_PREFIX"bumper", 50);
    }
    if (has_wheel_drop_)
    {
        wheeldrop_pub = rosnode_->advertise<ccr_msgs::BooleanSensor>(TOPIC_EVENT_PREFIX"wheel_drop", 50);
    }
    drive_pub = rosnode_->advertise<ccr_msgs::Drive>(TOPIC_EVENT_PREFIX"drive", 50);
	mode_pub = rosnode_->advertise<std_msgs::Byte>(TOPIC_EVENT_PREFIX"mode", 50);
    if (has_cleaning_module_)
    {
    	brushes_pub = rosnode_->advertise<ccr_msgs::Brushes>(TOPIC_EVENT_PREFIX"brushes", 50);
	    dirt_pub = rosnode_->advertise<ccr_msgs::DirtDetect>(TOPIC_EVENT_PREFIX"dirt_detect", 50);
	    dustbox_pub = rosnode_->advertise<ccr_msgs::BooleanSensor>(TOPIC_EVENT_PREFIX"dustbox", 50);
	    dust_pub = rosnode_->advertise<std_msgs::Byte>(TOPIC_EVENT_PREFIX"dust", 50);
    }
    if (has_led_board_)
    {
    	led_pub = rosnode_->advertise<ccr_msgs::LEDBoardEvent>(TOPIC_EVENT_PREFIX"led_board", 50);
    }
    if (has_ir_receiver_)
    {
	    irchar_pub = rosnode_->advertise<ccr_msgs::IRCharacter>(TOPIC_EVENT_PREFIX"ir_character", 50);
    }
    if (has_opt_left_ || has_opt_right_)
    {
        opticalRanging_pub = rosnode_->advertise<sensor_msgs::Range>(TOPIC_EVENT_PREFIX"optical_ranging_sensor", 50);
        if (has_opt_left_)
    	  laser_opt_left_pub = rosnode_->advertise<sensor_msgs::LaserScan>(TOPIC_EVENT_PREFIX"opt_left", 50);
        if (has_opt_right_)
	      laser_opt_right_pub = rosnode_->advertise<sensor_msgs::LaserScan>(TOPIC_EVENT_PREFIX"opt_right", 50);
    }
    if (has_us_left_ || has_us_right_ || has_us_front_ || has_us_back_)
    {
        ultrasonic_pub = rosnode_->advertise<sensor_msgs::Range>(TOPIC_EVENT_PREFIX"ultrasonic_sensor", 50);
        if (has_us_left_)
          laser_us_left_pub = rosnode_->advertise<sensor_msgs::LaserScan>(TOPIC_EVENT_PREFIX"us_left", 50);
        if (has_us_right_)
          laser_us_right_pub = rosnode_->advertise<sensor_msgs::LaserScan>(TOPIC_EVENT_PREFIX"us_right", 50);
        if (has_us_front_)
          laser_us_front_pub = rosnode_->advertise<sensor_msgs::LaserScan>(TOPIC_EVENT_PREFIX"us_front", 50);
        if (has_us_back_)
          laser_us_back_pub = rosnode_->advertise<sensor_msgs::LaserScan>(TOPIC_EVENT_PREFIX"us_back", 50);
    }
    if (has_slip_sensor_)
    {
        slip_pub = rosnode_->advertise<ccr_msgs::Slip>(TOPIC_EVENT_PREFIX"slip", 50);
    }
    if (has_cliff_)
    {
        cliff_pub = rosnode_->advertise<ccr_msgs::IRBumper>(TOPIC_EVENT_PREFIX"cliff", 50);
    }
	
	cmd_vel_sub  = rosnode_->subscribe<geometry_msgs::Twist>
		("cmd_vel", 1, &ccr::Ccr::cmdVelReceived, this);
	drive_pwm_sub  = rosnode_->subscribe<ccr_msgs::DrivePwm>
		(TOPIC_COMMAND_PREFIX"drive_pwm", 1, &ccr::Ccr::drivePwmReceived, this);
    mode_sub  = rosnode_->subscribe<std_msgs::String>
		(TOPIC_COMMAND_PREFIX"mode", 1, &ccr::Ccr::cmdModeReceived, this);
    imu_sub  = rosnode_->subscribe<sensor_msgs::Imu>
		("imu", 1, &ccr::Ccr::imuReceived, this);
    if (has_cleaning_module_)
    {
        brushes_sub = rosnode_->subscribe<ccr_msgs::Brushes_cmd>
          (TOPIC_COMMAND_PREFIX"brushes_cmd", 1, &ccr::Ccr::cmdBrushesReceived, this);
        brushes_pwm_sub = rosnode_->subscribe<ccr_msgs::BrushesPWM_cmd>
          (TOPIC_COMMAND_PREFIX"brushesPWM_cmd", 1, &ccr::Ccr::cmdBrushesPWMReceived, this);
    }
    if (has_ir_receiver_)
    {
        led_board_command_sub = rosnode_->subscribe<ccr_msgs::LEDBoardCommand>
          (TOPIC_COMMAND_PREFIX"ledboard_Command", 1, &ccr::Ccr::cmdLEDBoardReceived, this);
    }

	// ******************************************************************************************
	// initialize member parameters.
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	first_loop_=true;
	calibrate_vel_cnt_ = 0;	
	max_vel_x_ = 2.0;
	// ******************************************************************************************
	// start to listen to sensor data.
	mobile_base = boost::shared_ptr<ccr::DataPicker>(new ccr::SerialDataPicker(port.c_str()));
	if( mobile_base->init() == 0) ROS_INFO("Connected to Ccr.");
	else
	{
		ROS_FATAL("Could not connect to Ccr.[%s]", port.c_str());
		ROS_BREAK();
	}
	mobile_base->registerCB(boost::shared_ptr<DataPickerObserver>(this));

	return;	
}

// *****************************************************************************
// Handle sensor date update.
void ccr::Ccr::onUpdate()
{

	double last_x, last_y, last_yaw;
	double orientation_x, orientation_y, orientation_z = 0;
	double vel_x, vel_y, vel_yaw;
	double dt;
	float last_charge = 0.0;
	int time_remaining = -1;

	if (first_loop_)
	{
		mobile_base->calculateOdometry();
		mobile_base->resetOdometry();
		first_loop_ = false;
		return;
	}
        
	current_time = ros::Time::now();
			
	mobile_base->calculateOdometry();
	        
    vel_x = mobile_base->new_odometry_.getLinear();
    vel_y = 0.0;
    vel_yaw = mobile_base->new_odometry_.getAngular();
	
	// ******************************************************************************************
	//publish transforms
    if(publishTf)
    {
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = odom_frame_id;
        odom_trans.child_frame_id = base_frame_id;
        odom_trans.transform.translation.x = mobile_base->odometry_x_;
        odom_trans.transform.translation.y = mobile_base->odometry_y_;
        odom_trans.transform.translation.z = 0.0;
        if (use_imu_)
        {
            odom_trans.transform.rotation = imu_data_.orientation;
        } else {
            odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(mobile_base->odometry_yaw_);
        }
        tf_broadcaster.sendTransform(odom_trans);
    }

	// ******************************************************************************************
	//publish odometry
	nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = odom_frame_id;
	
		//set the position
		odom.pose.pose.position.x = mobile_base->odometry_x_;
		odom.pose.pose.position.y = mobile_base->odometry_y_;
		odom.pose.pose.position.z = 0.0;
        if (use_imu_)
        {
            odom.pose.pose.orientation = imu_data_.orientation;
        } else {
    		odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(mobile_base->odometry_yaw_);
        }
	    if(pose_cov_mat)
	        for(int i = 0; i < pose_covariance_matrix.size(); i++)
	            odom.pose.covariance[i] = pose_covariance_matrix[i];
	
		//set the velocity
		odom.child_frame_id = base_frame_id;
		odom.twist.twist.linear.x = vel_x;
		odom.twist.twist.linear.y = vel_y;
		odom.twist.twist.angular.z = vel_yaw;
	    if(twist_cov_mat)
	        for(int i = 0; i < pose_covariance_matrix.size(); i++)
	            odom.twist.covariance[i] = twist_covariance_matrix[i];
	    
		//publish the message
		odom_pub.publish(odom);

        double radius = vel_yaw;//mobile_base->actual_angular_velocity_;
    if (fabs(last_cmd_vel_.angular.z) > MIN_ROTATION) {
	if (fabs(last_cmd_vel_.angular.z) > fabs(radius * 2))
	{
		calibrate_vel_cnt_++;
		if (calibrate_vel_cnt_ > CALIB_THRESHOLD*2) calibrate_vel_cnt_--;
	} else {
		calibrate_vel_cnt_--;
		if (calibrate_vel_cnt_ < 0) calibrate_vel_cnt_ = 0;
	}
    }
	ROS_DEBUG("cur vel_cnt=%d\n", calibrate_vel_cnt_);

	// ******************************************************************************************
	//publish battery
    ccr_msgs::Battery battery;
		battery.header.stamp = current_time;
	    battery.voltage = mobile_base->voltage_;
	    battery.status = mobile_base->charging_state_;
	    battery.config_option = mobile_base->config_charging_option_;
	    battery.config_current = mobile_base->config_current_;
	    battery.config_charging_voltage = mobile_base->config_charging_voltage_;
	    battery.config_charging_current = mobile_base->config_charging_current_;
		if (has_battery_config_) battery_pub.publish(battery);

	// ******************************************************************************************	
	//publish bumpers
    ccr_msgs::Bumper bumper;
		bumper.left.header.stamp = current_time;
		bumper.left.state = mobile_base->bumper_[LEFT];
		bumper.right.header.stamp = current_time;
		bumper.right.state = mobile_base->bumper_[RIGHT];
		if (has_bumper_) bumper_pub.publish(bumper);

	// ******************************************************************************************
	//publish cliff
    ccr_msgs::IRBumper cliff;
		cliff.header.stamp = current_time;

		cliff.header.frame_id = "cliff_sensor_back_left";
		cliff.state = mobile_base->cliff_[LEFT];
		cliff.signal = (float)mobile_base->cliff_signal_[LEFT]/1000;
		if (has_cliff_) cliff_pub.publish(cliff);

		cliff.header.frame_id = "cliff_sensor_front_left";
		cliff.state = mobile_base->cliff_[FRONT_LEFT];
		cliff.signal = (float)mobile_base->cliff_signal_[FRONT_LEFT]/1000;
		if (has_cliff_) cliff_pub.publish(cliff);

		cliff.header.frame_id = "cliff_sensor_front_right";
		cliff.state = mobile_base->cliff_[FRONT_RIGHT];
		cliff.signal = (float)mobile_base->cliff_signal_[FRONT_RIGHT]/1000;
		if (has_cliff_) cliff_pub.publish(cliff);

		cliff.header.frame_id = "cliff_sensor_back_right";
		cliff.state = mobile_base->cliff_[RIGHT];
		cliff.signal = (float)mobile_base->cliff_signal_[RIGHT]/1000;
		if (has_cliff_) cliff_pub.publish(cliff);

		cliff.header.frame_id = "cliff_sensor_center";
		cliff.state = mobile_base->cliff_[CENTER];
		cliff.signal = (float)mobile_base->cliff_signal_[CENTER]/1000;

        if (has_cliff_) cliff_pub.publish(cliff);

	// ******************************************************************************************
	//publish wheeldrop
    ccr_msgs::BooleanSensor wheeldrop;
		wheeldrop.header.stamp = current_time;
		wheeldrop.header.frame_id = "wheel_left";
		wheeldrop.state = mobile_base->wheel_drop_[LEFT];
		if (has_wheel_drop_) wheeldrop_pub.publish(wheeldrop);

		wheeldrop.header.stamp = current_time;
		wheeldrop.header.frame_id = "wheel_right";
		wheeldrop.state = mobile_base->wheel_drop_[RIGHT];
		if (has_wheel_drop_) wheeldrop_pub.publish(wheeldrop);

    // ******************************************************************************************
    //publish infrared range
    sensor_msgs::Range opticalRange;
        opticalRange.header.stamp = current_time;

        opticalRange.header.frame_id = "optical_ranging_sensor_left";
        opticalRange.radiation_type = sensor_msgs::Range::INFRARED;
        opticalRange.field_of_view = 0.017453;
        opticalRange.min_range = 0.02;
        opticalRange.max_range = 0.15;

        opticalRange.range = (float)mobile_base->optical_range_[LEFT] / 1000;
        if (has_opt_left_) opticalRanging_pub.publish(opticalRange);

        opticalRange.header.frame_id = "optical_ranging_sensor_right";
        opticalRange.range = (float)mobile_base->optical_range_[RIGHT] / 1000;
        if (has_opt_right_) opticalRanging_pub.publish(opticalRange);

    sensor_msgs::LaserScan opt_laser;
        opt_laser.header.stamp = current_time;
        opt_laser.header.frame_id = "optical_ranging_sensor_left";
        opt_laser.angle_min = -0.1;
        opt_laser.angle_max = 0.1;
        opt_laser.angle_increment = 0.1;
        opt_laser.range_min = 0.02;
        opt_laser.range_max = 0.14;
        int opt_laser_cnt = (opt_laser.angle_max - opt_laser.angle_min) / opt_laser.angle_increment;
        opt_laser_cnt++;
        for (int i = 0; i < opt_laser_cnt; i++) {
            opt_laser.ranges.push_back((float)mobile_base->optical_range_[LEFT] / 1000);
        }
        if (has_opt_left_) laser_opt_left_pub.publish(opt_laser);

        opt_laser.header.frame_id = "optical_ranging_sensor_right";
        opt_laser.ranges.clear();
        for (int i = 0; i < opt_laser_cnt; i++) {
            opt_laser.ranges.push_back((float)mobile_base->optical_range_[RIGHT] / 1000);
        }
        if (has_opt_right_) laser_opt_right_pub.publish(opt_laser);

    // ******************************************************************************************
    //publish ultra range
    sensor_msgs::Range ultrasonic;
        ultrasonic.header.stamp = current_time;
        ultrasonic.radiation_type = sensor_msgs::Range::ULTRASOUND;
        ultrasonic.field_of_view = 0.5236;
        ultrasonic.min_range = 0.12;
        ultrasonic.max_range = 1.5;

        ultrasonic.header.frame_id = "ultrasonic_sensor_left";
        ultrasonic.range = (float)mobile_base->ultra_sonic_range_[LEFT]/1000;
        if (has_us_left_) ultrasonic_pub.publish(ultrasonic);

        ultrasonic.header.frame_id = "ultrasonic_sensor_right";
        ultrasonic.range = (float)mobile_base->ultra_sonic_range_[RIGHT]/1000;
        if (has_us_right_) ultrasonic_pub.publish(ultrasonic);

        ultrasonic.header.frame_id = "ultrasonic_sensor_front";
        ultrasonic.range = (float)mobile_base->ultra_sonic_range_[FRONT]/1000;
        if (has_us_front_) ultrasonic_pub.publish(ultrasonic);

        ultrasonic.header.frame_id = "ultrasonic_sensor_back";
        ultrasonic.range = (float)mobile_base->ultra_sonic_range_[BACK]/1000;
        if (has_us_back_) ultrasonic_pub.publish(ultrasonic);

    sensor_msgs::LaserScan us_laser;
        us_laser.header.stamp = current_time;
        us_laser.header.frame_id = "ultrasonic_sensor_left";
        us_laser.angle_min = -0.1;
        us_laser.angle_max = 0.1;
        us_laser.angle_increment = 0.1;
        us_laser.range_min = 0.05;
        us_laser.range_max = 1.0;
        //us_laser.scan_time = 20;
        int us_laser_cnt = (us_laser.angle_max - us_laser.angle_min) / us_laser.angle_increment;
        us_laser_cnt++;
        for (int i = 0; i < us_laser_cnt; i++) {
            us_laser.ranges.push_back((float)mobile_base->ultra_sonic_range_[LEFT] / 1000);
        }
        if (has_us_left_) laser_us_left_pub.publish(us_laser);

        us_laser.header.frame_id = "ultrasonic_sensor_right";
        us_laser.ranges.clear();
        for (int i = 0; i < us_laser_cnt; i++) {
            us_laser.ranges.push_back((float)mobile_base->ultra_sonic_range_[RIGHT] / 1000);
        }
        if (has_us_right_) laser_us_right_pub.publish(us_laser);

        us_laser.header.frame_id = "ultrasonic_sensor_front";
        us_laser.ranges.clear();
        for (int i = 0; i < us_laser_cnt; i++) {
            us_laser.ranges.push_back((float)mobile_base->ultra_sonic_range_[FRONT] / 1000);
        }
/*
        for (int i = 0; i < us_laser_cnt; i++) {
            us_laser.intensities.push_back(1.0);
        }
*/
        if (has_us_front_) laser_us_front_pub.publish(us_laser);

        us_laser.header.frame_id = "ultrasonic_sensor_back";
        us_laser.ranges.clear();
        for (int i = 0; i < us_laser_cnt; i++) {
            us_laser.ranges.push_back((float)mobile_base->ultra_sonic_range_[BACK] / 1000);
        }
        if (has_us_back_) laser_us_back_pub.publish(us_laser);
    // ******************************************************************************************
    //publish slip detect

    ccr_msgs::Slip slip;
            slip.header.stamp = current_time;
            slip.slip = mobile_base->slip_;
            if (has_slip_sensor_) slip_pub.publish(slip);


    // ******************************************************************************************
    //publish drive
    ccr_msgs::Drive drive;
        drive.header.stamp = current_time;
        drive.velocity = mobile_base->actual_velocity_;
        drive.radius = mobile_base->actual_angular_velocity_;
        drive.right_velocity = mobile_base->actual_right_velocity_;
        drive.left_velocity = mobile_base->actual_left_velocity_;
        drive.left_encoder_count = mobile_base->encoder_counts_[LEFT];
        drive.right_encoder_count = mobile_base->encoder_counts_[RIGHT];
        drive.left_motor_current = mobile_base->motor_current_[LEFT];
        drive.right_motor_current = mobile_base->motor_current_[RIGHT];
        drive_pub.publish(drive);

    // ******************************************************************************************
    //publish brushes
    ccr_msgs::Brushes brushes;
        brushes.header.stamp = current_time;
        brushes.brush_motor_current = mobile_base->motor_current_[MAIN_BRUSH];
        brushes.suction_motor_current = mobile_base->motor_current_[SACTION];
        brushes.brush_motor_encode_count = mobile_base->encoder_counts_[MAIN_BRUSH];
        brushes.suction_motor_encode_count = mobile_base->encoder_counts_[SACTION];
        if (has_cleaning_module_) brushes_pub.publish(brushes);

    // ******************************************************************************************
    //publish mode
	std_msgs::Byte mode;
		mode.data = mobile_base->mode_;
		mode_pub.publish(mode);

    // ******************************************************************************************
    //publish dust
	std_msgs::Byte dust;
		dust.data = mobile_base->dust_amount_;
		if (has_cleaning_module_) dust_pub.publish(dust);

    // ******************************************************************************************
    //publish dirt_detection
	ccr_msgs::DirtDetect dirt;
        dirt.header.stamp = current_time;
		dirt.dirt_high_level = mobile_base->dirt_detect_[DETECTION_HIGH];
		dirt.dirt_low_level = mobile_base->dirt_detect_[DETECTION_LOW];
		if (has_cleaning_module_) dirt_pub.publish(dirt);

    // ******************************************************************************************
    //publish dustbox
	ccr_msgs::BooleanSensor dbox;
        dbox.header.stamp = current_time;
		dbox.state = mobile_base->dustbox_;
		if (has_cleaning_module_) dustbox_pub.publish(dbox);

    // ******************************************************************************************
    //publish LED event.
	ccr_msgs::LEDBoardEvent led;
        led.header.stamp = current_time;
		led.event = mobile_base->led_board_event_[0];
		if (has_led_board_) led_pub.publish(led);

    // ******************************************************************************************
    //publish dustbox
	ccr_msgs::IRCharacter irchar;
        irchar.header.stamp = current_time;
		irchar.left = mobile_base->ir_char_[IR_LEFT][0];
		irchar.right = mobile_base->ir_char_[IR_RIGHT][0];
		irchar.center = mobile_base->ir_char_[IR_CENTER][0];
		irchar.omni = mobile_base->ir_char_[IR_OMNI][0];
		if (has_ir_receiver_) irchar_pub.publish(irchar);


	last_x = mobile_base->odometry_x_;
	last_y = mobile_base->odometry_y_;
	last_yaw = mobile_base->odometry_yaw_;
}

// *****************************************************************************
// Drive requirement.
void ccr::Ccr::cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
//--->for atomic serial access
#ifdef __ATOMIC_ACCESS__
std::lock_guard<std::mutex> lg(g_guard);
#endif
//<---for atomic serial access
	double x = cmd_vel->linear.x;
	if (calibrate_vel_cnt_ > CALIB_THRESHOLD && cmd_vel->linear.x > VEL_THRESHOLD)
	{
		max_vel_x_ *= 0.8;
		calibrate_vel_cnt_  = 0;
		ROS_WARN("--- over maximum speed---decrease[%f]->[%f]", cmd_vel->linear.x, max_vel_x_);
	}

    if (fabs(cmd_vel->angular.z) > MIN_ROTATION) {
	if (cmd_vel->linear.x > 0) {
		x = std::min(max_vel_x_, cmd_vel->linear.x);
	} else {
		x = std::max(-max_vel_x_, cmd_vel->linear.x);
	}
    }

	mobile_base->driveVelocity(x,cmd_vel->angular.z);

	last_cmd_vel_ = *cmd_vel;
	last_cmd_vel_.linear.x = x;
	
	
}

// *****************************************************************************
// Drive Pwm requirement.
void ccr::Ccr::drivePwmReceived(const ccr_msgs::DrivePwm::ConstPtr& drive_pwm)
{
//--->for atomic serial access
#ifdef __ATOMIC_ACCESS__
std::lock_guard<std::mutex> lg(g_guard);
#endif
//<---for atomic serial access
	mobile_base->drivePWM(drive_pwm->left,drive_pwm->right);
}


// *****************************************************************************
// Mode change requirement.
void ccr::Ccr::cmdModeReceived(const std_msgs::String::ConstPtr& cmd_)
{
//--->for atomic serial access
#ifdef __ATOMIC_ACCESS__
std::lock_guard<std::mutex> lg(g_guard);
#endif
//<---for atomic serial access
    if(cmd_->data=="reset_odom") mobile_base->resetOdometry();
	else if(cmd_->data==CCR_STATE_NORMAL)
	{
	    mobile_base->setMode(DataPicker::Mode::NORMAL);
	}
	else if(cmd_->data==CCR_STATE_MANUAL)
	{
	    mobile_base->setMode(DataPicker::Mode::MANUAL);
	}
}

// *****************************************************************************
// IMU data received.
void ccr::Ccr::imuReceived(const sensor_msgs::Imu::ConstPtr& _imu)
{
//--->for atomic serial access
#ifdef __ATOMIC_ACCESS__
std::lock_guard<std::mutex> lg(g_guard);
#endif
//<---for atomic serial access
    imu_data_ = *_imu;
}

// *****************************************************************************
// Brush action received.
void ccr::Ccr::cmdBrushesReceived(const ccr_msgs::Brushes_cmd::ConstPtr& cmd_)
{
//--->for atomic serial access
#ifdef __ATOMIC_ACCESS__
std::lock_guard<std::mutex> lg(g_guard);
#endif
//<---for atomic serial access
    mobile_base->brushes((uint8_t)cmd_->brush, (uint8_t)cmd_->vacuum, cmd_->brush, 0, 0);
}
void ccr::Ccr::cmdBrushesPWMReceived(const ccr_msgs::BrushesPWM_cmd::ConstPtr& cmd_)
{
//--->for atomic serial access
#ifdef __ATOMIC_ACCESS__
std::lock_guard<std::mutex> lg(g_guard);
#endif
//<---for atomic serial access
    mobile_base->brushesPWM(cmd_->brush, cmd_->brush, cmd_->vacuum);
}
void ccr::Ccr::cmdLEDBoardReceived(const ccr_msgs::LEDBoardCommand::ConstPtr& cmd_)
{
//--->for atomic serial access
#ifdef __ATOMIC_ACCESS__
std::lock_guard<std::mutex> lg(g_guard);
#endif
//<---for atomic serial access
    uint8_t buf[4];
    buf[0] = cmd_->cmd1;
    buf[1] = cmd_->cmd2;
    buf[2] = cmd_->cmd3;
    buf[3] = cmd_->cmd4;
    mobile_base->ledboardCommand(buf);
}

void ccr::Ccr::exportWheelRotation()
{
  mobile_base->exportWheelRotation();
}

// *****************************************************************************
// Main function.
int main(int argc, char** argv)
{
	ros::init(argc, argv, "mobile_base_node");

	ROS_INFO("co-creation robot for ROS %.2f", NODE_VERSION);

	ccr::Ccr ccr;
	ccr.init();

        ros::NodeHandle pn("~");
	double   encoder_logging_rate;
	pn.param<double>("encoder_logging_rate", encoder_logging_rate, CCR_ENCODER_LOGGING_RATE);

	ros::Rate r(encoder_logging_rate);
	while(ros::ok()) {
	  ccr.exportWheelRotation();
	  ros::spinOnce();
	  r.sleep();
	}
}

// EOF
