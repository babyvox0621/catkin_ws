/**
 * Copyright(C) 2017 Panasonic Corporation. All Right reserved.
 * 
 * \file        Ccr.cpp
 * \brief       Co-creation robot main class.
 * \date        2017/02/16  create a new
 *
 * \note        
 *
 */
#ifndef __CCR_H
#define __CCR_H

#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>

#include "diversion/odometry.h"
#include "DataPicker.h"
#include "DataPickerObserver.h"
#include "ccr_msgs/DrivePwm.h"
#include "ccr_msgs/Brushes_cmd.h"
#include "ccr_msgs/BrushesPWM_cmd.h"
#include "ccr_msgs/LEDBoardCommand.h"

namespace ccr
{
	/*! \class Ccr
	 *  \brief Ccr main class.
	 *
	 * Publish sensor data gathered by DataPicker.
	 */
	class Ccr : public DataPickerObserver
	{
		public:
        /***********************************************************************/
        /* functions */
        /***********************************************************************/
		//! Constructor.
		Ccr();

		//! Destructor.
		~Ccr();

		//! Initialization.
		void init();

		/*!
		 *  see class: DataPickerObserver::onUpdate()
		*/
		void onUpdate();

		//! cmd_vel callback.
		void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel);
		//! drive_pwm callback.
		void drivePwmReceived(const ccr_msgs::DrivePwm::ConstPtr& drive_pwm);
		//! mode change requirement callback.
		void cmdModeReceived(const std_msgs::String::ConstPtr& cmd_);
		//! imu callback.
		void imuReceived(const sensor_msgs::Imu::ConstPtr& _imu);
		//! Bushes action callback.
		void cmdBrushesReceived(const ccr_msgs::Brushes_cmd::ConstPtr& cmd_);
		void cmdBrushesPWMReceived(const ccr_msgs::BrushesPWM_cmd::ConstPtr& cmd_);
		//! LED action callback.
		void cmdLEDBoardReceived(const ccr_msgs::LEDBoardCommand::ConstPtr& cmd_);

		//! Export encoder count to syslog
		void exportWheelRotation();
	    /***********************************************************************/
        /* parameters */
        /***********************************************************************/
		private:
	    boost::shared_ptr<ros::NodeHandle> rosnode_;
		boost::shared_ptr<ccr::DataPicker> mobile_base;

		ros::Publisher odom_pub, battery_pub, bumper_pub, cliff_pub, wheeldrop_pub, 
			opticalRanging_pub, ultrasonic_pub, drive_pub, mode_pub, slip_pub, brushes_pub, 
            dirt_pub, dust_pub, dustbox_pub, led_pub, irchar_pub, laser_opt_left_pub, laser_opt_right_pub,
            laser_us_front_pub, laser_us_back_pub, laser_us_left_pub, laser_us_right_pub;
		ros::Subscriber cmd_vel_sub, drive_pwm_sub, mode_sub, imu_sub, brushes_sub,
            brushes_pwm_sub, led_board_command_sub;

		std::string base_frame_id;
		std::string odom_frame_id;

	    bool publishTf;
		tf::TransformBroadcaster tf_broadcaster;

		bool pose_cov_mat;
		bool twist_cov_mat;
		std::vector<double> pose_covariance_matrix; 
		std::vector<double> twist_covariance_matrix;

		ros::Time current_time, last_time;
		bool first_loop_;
        sensor_msgs::Imu imu_data_;
        bool use_imu_;
        bool has_cleaning_module_, has_led_board_, has_ir_receiver_, has_opt_left_
          , has_opt_right_, has_us_left_, has_us_right_, has_us_front_
          , has_us_back_, has_slip_sensor_, has_cliff_, has_wheel_drop_
          , has_bumper_, has_battery_config_;
    	int calibrate_vel_cnt_;
	    double max_vel_x_;
	    geometry_msgs::Twist last_cmd_vel_;
	};

}

#endif //__CCR_H

// EOF
