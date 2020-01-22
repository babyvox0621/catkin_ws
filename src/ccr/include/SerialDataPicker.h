/**
 * Copyright(C) 2017 Panasonic Corporation. All Right reserved.
 * 
 * \file        SerialDataPicker.h
 * \brief       Serial interface implemention for DataPicker.
 * \date        2017/02/16  create a new
 *
 * \note        
 *
 */
#ifndef __SERIAL_DATA_PICKER_H
#define __SERIAL_DATA_PICKER_H

#include <serial/serial.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <vector>

#include "DataPicker.h"
#include "diversion/odometry.h"

// Packets sizes
#define SENSOR_DATA_SIZE					103
#define SENSOR_BUMPS_DROPS_SIZE					1
#define SENSOR_CLIFF_LEFT_SIZE					1
#define SENSOR_CLIFF_FRONT_LEFT_SIZE				1
#define SENSOR_CLIFF_FRONT_RIGHT_SIZE			1
#define SENSOR_CLIFF_RIGHT_SIZE					1
#define SENSOR_CLIFF_CENTER_SIZE				1
#define SENSOR_VOLTAGE_SIZE						2
#define SENSOR_CHARGING_STATE_SIZE				1
#define SENSOR_CHARGING_CONFIG_SIZE				8
#define SENSOR_CLIFF_LEFT_SIGNAL_SIZE			2
#define SENSOR_CLIFF_FRONT_LEFT_SIGNAL_SIZE		2
#define SENSOR_CLIFF_FRONT_RIGHT_SIGNAL_SIZE		2
#define SENSOR_CLIFF_RIGHT_SIGNAL_SIZE			2
#define SENSOR_CLIFF_CENTER_SIGNAL_SIZE			2
#define SENSOR_ACTUAL_VELOCITY_SIZE			2
#define SENSOR_ACTUAL_ANGULAR_VELOCITY_SIZE			2
#define SENSOR_ACTUAL_RIGHT_VELOCITY_SIZE			2
#define SENSOR_ACTUAL_LEFT_VELOCITY_SIZE			2
#define SENSOR_RIGHT_ENCODER_SIZE				2
#define SENSOR_LEFT_ENCODER_SIZE					2
#define SENSOR_LEFT_MOTOR_CURRENT_SIZE			2	
#define SENSOR_RIGHT_MOTOR_CURRENT_SIZE			2
#define SENSOR_OPTICAL_RANGING_LEFT_SIZE			2
#define SENSOR_OPTICAL_RANGING_RIGHT_SIZE			2
#define SENSOR_ULTRASONIC_SIZE				8
#define SENSOR_SLIP_DETECT_SIZE				1
#define SENSOR_MODE_SIZE				1
#define SENSOR_BRUSHES_SIZE				4
#define SENSOR_SACTION_SIZE				4
#define SENSOR_DIRT_DETECTION_SIZE				2
#define SENSOR_DUSTBOX_DETECTION_SIZE				1
#define SENSOR_DUST_AMOUNT_SIZE				1
#define SENSOR_LED_BOARD_EVENT_SIZE				9
#define SENSOR_IR_CHAR_EACH_SIZE				6
#define SENSOR_RESERVE_SIZE				5


#define SERIAL_WAIT_TIME 20

#ifndef NORMALIZE
#define NORMALIZE(z) atan2(sin(z), cos(z))
#endif

#define DATA_HZ (20)
#define AXIS_THRESHOKD (0.6)

namespace ccr
{
	//! operation codes.
	/*!
	 * operation code to ccr.
	 */
	typedef enum Opcode {

		// Command opcodes
		SET_MODE = 0x01,
		DRIVE = 0x10,
		DRIVE_PWM = 0x11,
		MOTORS = 0x30,
		PWMMOTORS = 0x31,
		LEDCMD = 0x99,
	} Opcode;

	/*! \class SerialDataPicker
	 *  \brief serial interface implemention for DataPicker.
	 */
	class SerialDataPicker : public DataPicker
	{
		public:
	
		//! Constructor
		SerialDataPicker(const char * new_serial_port);
		//! Destructor
		virtual ~SerialDataPicker();

		/*!
		 *  see class: DataPicker::init()
		 */
		int init();
		/*!
		 *  see class: DataPicker::tearDown()
		 */
		int tearDown();
		/*!
		 *  see class: DataPicker::registerCB()
		 */
		void registerCB(boost::shared_ptr<DataPickerObserver> obj);

		/*!
		 *  see class: DataPicker::setMode()
		*/
		int setMode(uint8_t mode);

		/*!
		 *  see class: DataPicker::driveVelocity()
		*/
		int driveVelocity(double linear_speed, double angular_speed);
		/*!
		 *  see class: DataPicker::driveWheel()
		*/
		int driveWheel(int left_speed, int right_speed);
		/*!
		 *  see class: DataPicker::drivePWM()
		*/
		int drivePWM(int left_pwm, int right_pwm);
		/*!
		 *  see class: DataPicker::brushes()
		*/
		int brushes(unsigned char side_brush, unsigned char vacuum, unsigned char main_brush, unsigned char side_brush_clockwise, unsigned char main_brush_dir);
		/*!
		 *  see class: DataPicker::brushesPWM()
		*/
		int brushesPWM(uint8_t main_brush, uint8_t side_brush, uint8_t vacuum);
		/*!
		 *  see class: DataPicker::ledboardCommand()
		*/
		int ledboardCommand(uint8_t cmd[4]);
		/*!
		 *  see class: DataPicker::resetOdometry()
		*/
		void resetOdometry();
		/*!
		 *  see class: DataPicker::setOdometry()
		*/
		void setOdometry(double new_x, double new_y, double new_yaw);

		/*!
		 *  see class: DataPicker::calculateOdometry()
		*/
		void calculateOdometry();

		//! thread loop function.
		/*!
		 *  Continue to get sensor data in certain interval.
		 *  If getting data sccusess, call registered callback.
		*/
		void sensorDataLoop();

		/*!
		 *  Export abs(integral count of encoder) to syslog.
		 */
		void exportWheelRotation();

		private:
		SerialDataPicker();
	
		int parseSensorPackets(unsigned char * buffer, size_t buffer_length);
	
		int parseBumpersAndWheeldrops(unsigned char * buffer, int index);
		int parseMode(unsigned char * buffer, int index);
		int parseLeftCliff(unsigned char * buffer, int index);
		int parseFrontLeftCliff(unsigned char * buffer, int index);
		int parseFrontRightCliff(unsigned char * buffer, int index);
		int parseRightCliff(unsigned char * buffer, int index);	
		int parseVoltage(unsigned char * buffer, int index);
		int parseChargingState(unsigned char * buffer, int index);
		int parseChargingConfig(unsigned char * buffer, int index);
		int parseLeftCliffSignal(unsigned char * buffer, int index);
		int parseFrontLeftCliffSignal(unsigned char * buffer, int index);
		int parseFontRightCliffSignal(unsigned char * buffer, int index);
		int parseRightCliffSignal(unsigned char * buffer, int index);
		int parseActualVelocity(unsigned char * buffer, int index);
		int parseActualAngularVelocity(unsigned char * buffer, int index);
		double parseActualRightVelocity(unsigned char * buffer, int index);
		double parseActualLeftVelocity(unsigned char * buffer, int index);
		int parseRightEncoderCounts(unsigned char * buffer, int index);
		int parseLeftEncoderCounts(unsigned char * buffer, int index);
		int parseLeftMotorCurrent(unsigned char * buffer, int index);
		int parseRightMotorCurrent(unsigned char * buffer, int index);
		int parseCenterCliffSignal(unsigned char * buffer, int index);
		int parseCenterCliff(unsigned char * buffer, int index);	
		int parseOpticalRangingLeft(unsigned char * buffer, int index);
		int parseOpticalRangingRight(unsigned char * buffer, int index);
		int parseUltraSonic(unsigned char * buffer, int index);
		int parseSlipDetect(unsigned char * buffer, int index);
		int parseBrushes(unsigned char * buffer, int index);
		int parseSaction(unsigned char * buffer, int index);
		int parseDirtDetect(unsigned char * buffer, int index);
		int parseDustboxDetect(unsigned char * buffer, int index);
		int parseDustAmount(unsigned char * buffer, int index);
		int parseLedBoard(unsigned char * buffer, int index);
		int parseIrChar(unsigned char * buffer, int index);
	
		int buffer2signed_int(unsigned char * buffer, int index);
		int buffer2unsigned_int(unsigned char * buffer, int index);
		int sendOpcode(Opcode code);

		void logger(const char* p_Fmt, ...);
	
		//! Serial port to which the robot is connected
		std::string port_name_;
		//! Serial port object
		serial::Serial serial_port_;
		//! Total size of packets
		size_t packets_size_;
		//! loop thread for packet reading.
		boost::thread read_thread_;
		//! flag for loop.
		bool loop_;
		float wheel_diameter_;
		float axle_length_;
		std::vector<double> left_encoders;
		std::vector<double> right_encoders;
		//! Integral value of encoder
		long integ_left_encoder;
		long integ_right_encoder;

	};

}

#endif //__SERIAL_DATA_PICKER_H

// EOF
