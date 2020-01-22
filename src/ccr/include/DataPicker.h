/**
 * Copyright(C) 2017 Panasonic Corporation. All Right reserved.
 * 
 * \file        DataPicker.h
 * \brief       Super class for robot controll.
 * \date        2017/02/16  create a new
 *
 * \note        
 *
 */
#ifndef __DATA_PICKER_H
#define __DATA_PICKER_H

#include "diversion/odometry.h"
#include "DataPickerObserver.h"

namespace ccr
{
	/*! \class DataPicker
	 *  \brief Super class for robot controll.
	 *
	 * This class is to be implemented by actual interface, for example Serial, TCP ... etc.
	 */
	class DataPicker
	{
		public:

		//! mode.
		typedef enum  {
			NORMAL = 1,
		    MANUAL = 2,
			FORCE_STOP = 3
		} Mode;
	
        /***********************************************************************/
        /* functions */
        /***********************************************************************/
		//! Constructor
		DataPicker():new_odometry_() {};
		//! Destructor
		virtual ~DataPicker() {};
	
		//! Prepare using interfaces.
		virtual int init() = 0;

		//! finish using interfaces.
		virtual int tearDown() = 0;
	
		//! Set mode.
		/*!
		*  set requested mode.
		*  Tipically used when reset the signal for emergency stop.
        *
		*  \param mode		mode. see ::Mode .
		*
		*/
		virtual int setMode(uint8_t mode) = 0;

		//! Register callback object.
		/*!
		*  Register callback object.
        *
		*  \param obj		Callback object.
		*
		*/
		virtual void registerCB(boost::shared_ptr<DataPickerObserver> obj) = 0;

		//! Operate drive motion(1).
		/*!
		*  Send velocity commands to rulo with left-right wheel speed.
		*
		*  \param linear_speed  	linear speed.
		*  \param angular_speed  	angular speed.
		*
		*  \return 0 if ok, -1 otherwise.
		*/
		virtual int driveVelocity(double linear_speed, double angular_speed) = 0;

		//! Operate drive motion(2).
		/*!
		*  Send velocity commands to rulo with left-right wheel speed.
		*
		*  \param left_speed  	Left wheel speed.
		*  \param right_speed  	Right wheel speed.
		*
		*  \return 0 if ok, -1 otherwise.
		*/
		virtual int driveWheel(int left_speed, int right_speed) = 0;

		//! Operate drive motion(3).
		/*!
		*  Send velocity commands to rulo with left-right PWM.
		*
		*  \param left_pwm  	left pwm.
		*  \param right_pwm  	right pwm.
		*
		*  \return 0 if ok, -1 otherwise.
		*/
		virtual int drivePWM(int left_pwm, int right_pwm) = 0;
		//! Set brushes
		/*!
		*  Set the various brushes motors.
		*
		*  \param side_brush  			Side brush on (1) or off (0).
		*  \param vacuum  				Vacuum on (1) or off (0).
		*  \param main_brush 			Main brush on (1) or off (0).
		*  \param side_brush_clockwise 	Wether to rotate the side brush clockwise or not.
		*  \param main_brush_dir 		Main brush direction.
		*
		*  \return 0 if ok, -1 otherwise.
		*/
		virtual int brushes(unsigned char side_brush, unsigned char vacuum, unsigned char main_brush, unsigned char side_brush_clockwise, unsigned char main_brush_dir) = 0;
		//! Set brushes motors pwms
		/*!
		*  Set the brushes motors pwms.
		*
		*  \param main_brush 	Main brush motor pwm.
		*  \param side_brush  	Side brush motor pwm.
		*  \param vacuum  		Vacuum motor pwm.
		*
		*  \return 0 if ok, -1 otherwise.
		*/
		virtual int brushesPWM(uint8_t main_brush, uint8_t side_brush, uint8_t vacuum) = 0;
		
		virtual int ledboardCommand(uint8_t cmd[4]) = 0;

		//! Reset calculated odometry.
		virtual void resetOdometry() = 0;
		//! Set indicated odometry.
		virtual void setOdometry(double new_x, double new_y, double new_yaw) = 0;
		//! Calculate Roomba odometry. Call after reading encoder pulses.
		virtual void calculateOdometry() = 0;

		//! Export abs(integral count of encoder) to syslog.
		virtual void exportWheelRotation() = 0;

	    /***********************************************************************/
        /* parameters */
        /***********************************************************************/

		//! odometry coodinate x
		double odometry_x_;
		//! odometry coodinate y
		double odometry_y_;
		//! odometry rotation yaw
		double odometry_yaw_;
	
		Odometry new_odometry_;  //NEW ODOMETRY
        
		bool cliff_[5];					//! Cliff sensors. Indexes: LEFT FRONT_LEFT FRONT_RIGHT RIGHT CENTER
		bool bumper_[2];				//! Bumper sensors. Indexes: LEFT RIGHT
		bool wheel_drop_[2];			//! Wheel drop sensors: Indexes: LEFT RIGHT
		int cliff_signal_[5];			//! CLiff sensors signal. Indexes: LEFT FRONT_LEFT FRONT_RIGHT RIGHT CENTER
		int32_t motor_current_[4];			//! Motor current. Indexes: LEFT RIGHT MAIN_BRUSH SACTION
		float voltage_;					//! Battery voltage in volts.
		uint8_t charging_state_;	//! Charging state. 0 : Not charge, 1: charge
		uint32_t config_charging_option_;		//! Charging option.
		int32_t config_current_;		//! desired current.
		int32_t config_charging_current_;		//! desired charging current.
		int32_t config_charging_voltage_;		//! desired charging voltage.
		int32_t optical_range_[2];			//! Range of infrared sonar.
		int32_t ultra_sonic_range_[4];		//! Range of ultra sonar.Indexes: LEFT RIGHT FRONT BACK
		double actual_velocity_;			//! Actual Velocity measured in base board.
		double actual_angular_velocity_;			//! Actual Angular Velocity measured in base board.
		double actual_right_velocity_;		//! Actual right wheel elocity measured in base board.
		double actual_left_velocity_;			//! Actual left wheel velocity measured in base board.
		int32_t encoder_counts_[4];//! Delta encoder counts. Indexes: LEFT RIGHT MAIN_BRUSH SACTION
		uint16_t last_encoder_counts_[2];//! Last encoder counts reading. For odometry calculation.
		int32_t max_encoder_counts_;       //! maximum of encoder
		int32_t max_cycle_encoder_counts_; //! encoder count of cycle
		uint32_t integral_encoder_[2];   //! Integral value of encorder
		int32_t encoder_threshold_;
		bool slip_; //! hole sensor for slip detection.
		int8_t mode_; //! current Rulo's mode.
		uint8_t dirt_detect_[2];	//! Dirt detection level. Indexes: DETECTION_LOW, HIGH
		uint8_t dustbox_;			//! dust box status. 0 : not detect, 1: detected.
		uint8_t dust_amount_;			//! dust amount level.
		uint8_t led_board_event_[9];			//! LED board event.
		uint8_t ir_char_[4][6];			//! IR character event.

		boost::shared_ptr<DataPickerObserver> observer_; //! callback object.
	};

}

#endif //__DATA_PICKER_H

// EOF
