/*
 * Serial interface implemention for DataPicker.
 * Copyright(c) panasonic.com
 */
/**
 * Copyright(C) 2017 Panasonic Corporation. All Right reserved.
 * 
 * \file        SerialDataPicker.cpp
 * \brief       Serial interface implemention for DataPicker.
 * \date        2017/02/16  create a new
 *
 * \note        If this is not suitable for ceatain STM version,
 *              Rulo does not move at all.
 */
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <string>
#include <numeric>
#include <netinet/in.h>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>
#include <syslog.h>
#include <stdarg.h>
#include "SerialDataPicker.h"
#include "diversion/odometry.h"
#include "Ccr_Constants.h"

#include <ros/ros.h>

#define SAMPLE_COUNT (5)
// *****************************************************************************
// Constructor
ccr::SerialDataPicker::SerialDataPicker()
{
}

ccr::SerialDataPicker::SerialDataPicker(const char * new_serial_port)
{	
	port_name_ = new_serial_port;
	
	encoder_counts_[LEFT] = -1;
	encoder_counts_[RIGHT] = -1;
	
	last_encoder_counts_[LEFT] = 0;
	last_encoder_counts_[RIGHT] = 0;

	integral_encoder_[LEFT] = 0;
	integral_encoder_[RIGHT] = 0;
	
	packets_size_ = SENSOR_DATA_SIZE;
        ros::NodeHandle pn("~");
        pn.param<float>("wheel_diameter", wheel_diameter_, CCR_WHEEL_DIAMETER);
        pn.param<float>("axle_length", axle_length_, CCR_AXLE_LENGTH);
	pn.param<int32_t>("max_encoder_counts", max_encoder_counts_, CCR_MAX_ENCODER_COUNTS);
	pn.param<int32_t>("max_cycle_encoder_counts", max_cycle_encoder_counts_, CCR_CYCLE_ENCODER_COUNTS);
	pn.param<int32_t>("encoder_threshold", encoder_threshold_, CCR_ENCODER_LOGGING_THRESHOLD);
        ROS_INFO("wheel_diameter           %f", wheel_diameter_);
        ROS_INFO("axle_length              %f", axle_length_);
        ROS_INFO("max_encoder_counts       %d", max_encoder_counts_);
	ROS_INFO("max_cycle_encoder_counts %d", max_cycle_encoder_counts_);
        ROS_INFO("encoder_threshold        %d", encoder_threshold_);

   
	this->new_odometry_.setWheelParams(axle_length_, wheel_diameter_/2.0);
	this->new_odometry_.init(ros::Time::now());
}


// *****************************************************************************
// Destructo
ccr::SerialDataPicker::~SerialDataPicker()
{
	// NOP
}


// *****************************************************************************
// Open the serial port
int ccr::SerialDataPicker::init()
{
	try
	{ 
        serial_port_.setPort(port_name_);
        //serial_port_.setBaudrate(57600);
        serial_port_.setBaudrate(115200);
        serial_port_.open();
		loop_ = true;
        read_thread_ = boost::thread(&ccr::SerialDataPicker::sensorDataLoop, this);
    } catch(std::exception& e) {
        std::cerr<<"Rulo Error opening serial port: "<<e.what()<<std::endl;
        return(-1); 
    }
    
	return(0);
}


// *****************************************************************************
// Close the serial port
int ccr::SerialDataPicker::tearDown()
{
	this->driveVelocity(0.0, 0.0);
	usleep(SERIAL_WAIT_TIME * 1e3);

	loop_ = false;
    read_thread_.join();

    try {
        serial_port_.close();
	} catch(std::exception& e) {
        std::cerr<<e.what()<<std::endl;
        return(-1);
	}
	ROS_DEBUG("end!\n");
	return(0);
}

// *****************************************************************************
// Close the serial port
void ccr::SerialDataPicker::registerCB(boost::shared_ptr<DataPickerObserver> obj)
{
    observer_ = obj;
}

// *****************************************************************************
// Send an OP code to the CCR
int ccr::SerialDataPicker::sendOpcode(Opcode code)
{
	uint8_t to_send = code;
	std::cout<<"Sending OPCode: "<<(uint8_t)to_send<<std::endl;
	try{ serial_port_.write(&to_send, 1); }
	catch(std::exception& e){
        std::cerr<<e.what()<<std::endl;
        return(-1);}
	return(0);
}


// *****************************************************************************
// Start the CCR
int ccr::SerialDataPicker::setMode(uint8_t mode)
{
	// Compose comand
	uint8_t cmd_buffer[2];
	cmd_buffer[0] = SET_MODE;
	cmd_buffer[1] = mode;

    ROS_DEBUG("-----setMode[%02x][%02x]---\n", cmd_buffer[0],cmd_buffer[1]);

    serial_port_.write(cmd_buffer, 2);

	return (0);
}

// *****************************************************************************
// Set the speeds
int ccr::SerialDataPicker::driveVelocity(double linear_speed, double angular_speed)
{
	int left_speed_mm_s = (int)((linear_speed-axle_length_*angular_speed/2)*1e3);		// Left wheel velocity in mm/s
	int right_speed_mm_s = (int)((linear_speed+axle_length_*angular_speed/2)*1e3);	// Right wheel velocity in mm/s
	
	return this->driveWheel(left_speed_mm_s, right_speed_mm_s);
}


// *****************************************************************************
// Set the wheel speeds
int ccr::SerialDataPicker::driveWheel(int left_speed, int right_speed)
{
	// prohibit opposite drive
/*
	if (left_speed > 0 && right_speed < 0) {
		right_speed = 0;
        } else if (left_speed < 0 && right_speed > 0) {
		left_speed = 0;
        } 
*/
	// Limit velocity
	int16_t left_speed_mm_s = left_speed; //std::max(left_speed, -CCR_MAX_LIN_VEL_MM_S);
	//int16_t left_speed_mm_s = std::max(left_speed, -CCR_MAX_LIN_VEL_MM_S);
	//left_speed_mm_s = std::min(left_speed, CCR_MAX_LIN_VEL_MM_S);
	int16_t right_speed_mm_s = right_speed; //std::max(right_speed, -CCR_MAX_LIN_VEL_MM_S);
	//int16_t right_speed_mm_s = std::max(right_speed, -CCR_MAX_LIN_VEL_MM_S);
	//right_speed_mm_s = std::min(right_speed, CCR_MAX_LIN_VEL_MM_S);
	
	// Compose comand
	uint8_t cmd_buffer[5];
	cmd_buffer[0] = DRIVE;
	cmd_buffer[1] = (left_speed_mm_s >> 8);
	cmd_buffer[2] = (left_speed_mm_s & 0xFF);
	cmd_buffer[3] = (right_speed_mm_s >> 8);
	cmd_buffer[4] = (right_speed_mm_s & 0xFF);

        ROS_DEBUG("-----driveWheel[%02x][%02x][%02x][%02x][%02x]---\n", cmd_buffer[0],cmd_buffer[1],cmd_buffer[2],cmd_buffer[3],cmd_buffer[4]);


            serial_port_.write(cmd_buffer, 5);

	return(0);
}


// *****************************************************************************
// Set the motor PWMs
int ccr::SerialDataPicker::drivePWM(int left_pwm, int right_pwm)
{
	// Compose comand
	uint8_t cmd_buffer[5];
	cmd_buffer[0] = DRIVE_PWM;
	cmd_buffer[1] = (left_pwm >> 8);
	cmd_buffer[2] = (left_pwm & 0xFF);
	cmd_buffer[3] = (right_pwm >> 8);
	cmd_buffer[4] = (right_pwm & 0xFF);

        ROS_DEBUG("-----drivePWM[%02x][%02x][%02x][%02x][%02x]---\n", cmd_buffer[0],cmd_buffer[1],cmd_buffer[2],cmd_buffer[3],cmd_buffer[4]);


            serial_port_.write(cmd_buffer, 5);
	return(0);
}

// *****************************************************************************
// Set the brushes motors status
int ccr::SerialDataPicker::brushes(unsigned char side_brush, unsigned char vacuum, unsigned char main_brush, unsigned char side_brush_clockwise, unsigned char main_brush_dir)
{
	uint8_t cmd_buffer[2];
	cmd_buffer[0] = MOTORS;
	cmd_buffer[1] = side_brush | vacuum<<1 | main_brush<<2 | side_brush_clockwise<<3 | main_brush_dir<<4;
	
        ROS_DEBUG("-----MOTORS[%02x][%02x]---\n", cmd_buffer[0],cmd_buffer[1]);

#if 0
for (int i = 0; i < sizeof(cmd_buffer); i++) {
        serial_port_.write(cmd_buffer+i, 1);

	uint8_t rcv_buffer[2];
        memset(rcv_buffer, 0xFF, sizeof(rcv_buffer));
        serial::Timeout T = serial::Timeout::simpleTimeout(100);
        serial_port_.setTimeout(T);
        bool readable=/*true;*/serial_port_.waitReadable();
        if (readable) {
            serial_port_.read(rcv_buffer, sizeof(rcv_buffer)); 
        printf("-----rcv[%02x][%02x]---\n", rcv_buffer[0],rcv_buffer[1]);
       } else {
	printf("cannot rcv!!\n");
       }
}
#endif

	try{ serial_port_.write((uint8_t*)cmd_buffer, 2); }
	catch(std::exception& e){
        std::cerr<<e.what()<<std::endl;
        return(-1);}

    return(0);
}

// *****************************************************************************
// Set the brushes motors PWMs
int ccr::SerialDataPicker::brushesPWM(uint8_t main_brush, uint8_t side_brush, uint8_t vacuum)
{
	uint8_t cmd_buffer[4];
	cmd_buffer[0] = PWMMOTORS;
	cmd_buffer[1] = main_brush;
	cmd_buffer[2] = side_brush;
	cmd_buffer[3] = vacuum<0 ? 0 : vacuum;

	try{ serial_port_.write(cmd_buffer, 4); }
	catch(std::exception& e){
        std::cerr<<e.what()<<std::endl;
        return(-1);}
	return(0);
}

// *****************************************************************************
// Set the LED Board Command
int ccr::SerialDataPicker::ledboardCommand(uint8_t cmd[4])
{
	uint8_t cmd_buffer[10];
    memset(cmd_buffer, 0x00, sizeof(cmd_buffer));
	cmd_buffer[0] = LEDCMD;
	cmd_buffer[1] = cmd[0];
	cmd_buffer[2] = cmd[1];
	cmd_buffer[3] = cmd[2];
	cmd_buffer[4] = cmd[3];
	
        ROS_DEBUG("-----LED_BOARD[%02x][%02x][%02x][%02x][%02x][%02x][%02x][%02x][%02x][%02x]---\n", cmd_buffer[0],cmd_buffer[1],cmd_buffer[2],cmd_buffer[3],cmd_buffer[4],cmd_buffer[5],cmd_buffer[6],cmd_buffer[7],cmd_buffer[8],cmd_buffer[9]);

	try{ serial_port_.write(cmd_buffer, 10); }
	catch(std::exception& e){
        std::cerr<<e.what()<<std::endl;
        return(-1);}
	return(0);
}

// *****************************************************************************
// Read the sensors
//#define __REQ_COMMAND__

void ccr::SerialDataPicker::sensorDataLoop()
{
	uint8_t data_buffer[packets_size_];

	ros::Rate r(60.0); // enough to large value for STM's publish rate.
    int timeout = 1000 / 50.0;

	uint8_t cmd_buffer[2];
	cmd_buffer[0] = 0x20;			// Subscibe
	cmd_buffer[1] = 1;				// Enable
	serial_port_.write(cmd_buffer, 2);

	size_t current_data_num = 0;

	while (loop_)
    {
#ifdef __REQ_COMMAND__
		uint8_t cmd_buffer2[3];
		cmd_buffer2[0] = 149;			// Query
		cmd_buffer2[1] = 1;				// Number of packets
		cmd_buffer2[2] = 100;			// GROUP_ALL
		serial_port_.write(cmd_buffer2, 3);
#endif

		try { 
		    serial::Timeout T = serial::Timeout::simpleTimeout(timeout);
		    serial_port_.setTimeout(T);
			serial_port_.waitByteTimes(packets_size_);

	        current_data_num += serial_port_.read(
				data_buffer + current_data_num
				, packets_size_ - current_data_num);

			int limit = 0;
	        for (limit = 0; current_data_num < packets_size_ && limit < 10; limit++)
			{
	            current_data_num += serial_port_.read(
					data_buffer + current_data_num
					, packets_size_ - current_data_num);
				ROS_INFO("[INFO]Thre is remained data.\n");
				usleep(1 * 1e3);	// 1ms
	        }
			if (limit >= 10)
			{
				ROS_INFO("[INFO]couldn't get all data -> retry subscribe.\n");
				serial_port_.write(cmd_buffer, 2);
				usleep(SERIAL_WAIT_TIME * 1e3);
				continue;
			} else {
				current_data_num = 0;
			}

			uint8_t outbuf[5*packets_size_];
			for (int i = 0; i < packets_size_; i++){
				sprintf((char*)(outbuf + (i * 4)),"[%02x]", data_buffer[i]);
			}
			ROS_DEBUG("%s", outbuf);

		} catch(std::exception& e){
				std::cerr<<e.what()<<std::endl;
				ROS_WARN("error occurred.\n");
				continue;
		}
		    
		int ret = this->parseSensorPackets(/*(unsigned char*)*/data_buffer, packets_size_);
		if (ret < 0) {
			cmd_buffer[1] = 0;				// Disable
			serial_port_.write(cmd_buffer, 2);
			sensorDataLoop();
			return;
		}

		if (this->observer_ != NULL)
		{
			this->observer_->onUpdate();
		}

		ros::spinOnce();
		r.sleep();
    }

	cmd_buffer[1] = 0;				// Disable
	serial_port_.write(cmd_buffer, 2);

}


// *****************************************************************************
// Parse sensor data
int ccr::SerialDataPicker::parseSensorPackets(unsigned char * buffer , size_t buffer_lenght)
{	
	if(buffer_lenght != packets_size_)
	{
		// Error wrong packet size
		return(-1);
	}

	int i = 0;
	unsigned int index = 0;
	while(index < packets_size_)
	{
			index += parseBumpersAndWheeldrops(buffer, index);
			index += parseMode(buffer, index);
			// if mode is not defined mode, buffer is unreliable.
			if (this->mode_ < 1 || this->mode_ > 3) return -1;
			index += parseLeftCliff(buffer, index);
			index += parseFrontLeftCliff(buffer, index);
			index += parseFrontRightCliff(buffer, index);
			index += parseRightCliff(buffer, index);
            index += parseCenterCliff(buffer, index);
			index += parseLeftCliffSignal(buffer, index);
			index += parseFrontLeftCliffSignal(buffer, index);
			index += parseFontRightCliffSignal(buffer, index);
			index += parseRightCliffSignal(buffer, index);
            index += parseCenterCliffSignal(buffer, index);
			index += parseVoltage(buffer, index);
			index += parseChargingState(buffer, index);
			index += parseChargingConfig(buffer, index);
			index += parseActualLeftVelocity(buffer, index);
			index += parseActualRightVelocity(buffer, index);
			index += parseLeftEncoderCounts(buffer, index);
			index += parseRightEncoderCounts(buffer, index);
			index += parseLeftMotorCurrent(buffer, index);
			index += parseRightMotorCurrent(buffer, index);
            index += parseSlipDetect(buffer, index);
            index += parseOpticalRangingLeft(buffer, index);
            index += parseOpticalRangingRight(buffer, index);
            index += parseUltraSonic(buffer, index);
            index += parseBrushes(buffer, index);
            index += parseSaction(buffer, index);
            index += parseDirtDetect(buffer, index);
            index += parseDustboxDetect(buffer, index);
            index += parseDustAmount(buffer, index);
            index += parseLedBoard(buffer, index);
            index += parseIrChar(buffer, index);
            index += SENSOR_RESERVE_SIZE;
			i++;
	}
	this->actual_velocity_ = (this->actual_left_velocity_ + this->actual_right_velocity_) / 2;
	this->actual_angular_velocity_ = -(this->actual_right_velocity_ - this->actual_left_velocity_) / (axle_length_);
	return(0);
}

int ccr::SerialDataPicker::parseBumpersAndWheeldrops(unsigned char * buffer, int index)
{
	// Bumps, wheeldrops	
	this->bumper_[RIGHT] = (buffer[index]) & 0x01;
	this->bumper_[LEFT] = (buffer[index] >> 1) & 0x01;
	this->wheel_drop_[RIGHT] = (buffer[index] >> 2) & 0x01;
	this->wheel_drop_[LEFT] = (buffer[index] >> 3) & 0x01;
	
	return SENSOR_BUMPS_DROPS_SIZE;
}

int ccr::SerialDataPicker::parseMode(unsigned char * buffer, int index)
{
	// mode.
	this->mode_ = buffer[index];
	
	return SENSOR_MODE_SIZE;
}
	
int ccr::SerialDataPicker::parseLeftCliff(unsigned char * buffer, int index)
{
	// Cliffs
	this->cliff_[LEFT] = buffer[index] & 0x01;
	
	return SENSOR_CLIFF_LEFT_SIZE;
}

int ccr::SerialDataPicker::parseFrontLeftCliff(unsigned char * buffer, int index)
{
	// Cliffs
	this->cliff_[FRONT_LEFT] = buffer[index] & 0x01;
	
	return SENSOR_CLIFF_FRONT_LEFT_SIZE;
}

int ccr::SerialDataPicker::parseFrontRightCliff(unsigned char * buffer, int index)
{
	// Cliffs
	this->cliff_[FRONT_RIGHT] = buffer[index] & 0x01;
	
	return SENSOR_CLIFF_FRONT_RIGHT_SIZE;
}

int ccr::SerialDataPicker::parseRightCliff(unsigned char * buffer, int index)
{
	// Cliffs
	this->cliff_[RIGHT] = buffer[index] & 0x01;
	
	return SENSOR_CLIFF_RIGHT_SIZE;
}

int ccr::SerialDataPicker::parseVoltage(unsigned char * buffer, int index)
{
	// Voltage
	this->voltage_ = (float)(buffer2unsigned_int(buffer, index) / 1000.0);

	return SENSOR_VOLTAGE_SIZE;
}

int ccr::SerialDataPicker::parseChargingState(unsigned char * buffer, int index)
{
	// Charging State
	unsigned char byte = buffer[index];
	
	this->charging_state_ = byte;

	return SENSOR_CHARGING_STATE_SIZE;
}

int ccr::SerialDataPicker::parseChargingConfig(unsigned char * buffer, int index)
{
	// Charging Configs
	this->config_charging_option_ = buffer2unsigned_int(buffer, index);
	this->config_current_ = buffer2signed_int(buffer, index + 2);
	this->config_charging_current_ = buffer2signed_int(buffer, index + 4);
	this->config_charging_voltage_ = buffer2signed_int(buffer, index + 6);

	return SENSOR_CHARGING_CONFIG_SIZE;
}

int ccr::SerialDataPicker::parseLeftCliffSignal(unsigned char * buffer, int index)
{
	// Cliff signals
	this->cliff_signal_[LEFT] = buffer2unsigned_int(buffer, index);
	
	return SENSOR_CLIFF_LEFT_SIGNAL_SIZE;
}

int ccr::SerialDataPicker::parseFrontLeftCliffSignal(unsigned char * buffer, int index)
{
	// Cliff signals
	this->cliff_signal_[FRONT_LEFT] = buffer2unsigned_int(buffer, index);
	
	return SENSOR_CLIFF_FRONT_LEFT_SIGNAL_SIZE;
}
	
int ccr::SerialDataPicker::parseFontRightCliffSignal(unsigned char * buffer, int index)
{
	// Cliff signals
	this->cliff_signal_[FRONT_RIGHT] = buffer2unsigned_int(buffer, index);
	
	return SENSOR_CLIFF_FRONT_RIGHT_SIGNAL_SIZE;
}

int ccr::SerialDataPicker::parseRightCliffSignal(unsigned char * buffer, int index)
{
	// Cliff signals
	this->cliff_signal_[RIGHT] = buffer2unsigned_int(buffer, index);
	
	return SENSOR_CLIFF_RIGHT_SIGNAL_SIZE;
}

int ccr::SerialDataPicker::parseActualVelocity(unsigned char * buffer, int index)
{
    this->actual_velocity_ = buffer2unsigned_int(buffer, index);
	return SENSOR_ACTUAL_VELOCITY_SIZE;
}

int ccr::SerialDataPicker::parseActualAngularVelocity(unsigned char * buffer, int index)
{
    this->actual_angular_velocity_ = buffer2unsigned_int(buffer, index);
	return SENSOR_ACTUAL_ANGULAR_VELOCITY_SIZE;
}

double ccr::SerialDataPicker::parseActualRightVelocity(unsigned char * buffer, int index)
{
    this->actual_right_velocity_ = buffer2signed_int(buffer, index);
    // correct mm/s -> m/sec
    this->actual_right_velocity_ /= (10 * 100);
	return SENSOR_ACTUAL_RIGHT_VELOCITY_SIZE;
}

double ccr::SerialDataPicker::parseActualLeftVelocity(unsigned char * buffer, int index)
{
    this->actual_left_velocity_ = buffer2signed_int(buffer, index);
    // correct mm/s -> m/sec
    this->actual_left_velocity_ /= (10 * 100);
	return SENSOR_ACTUAL_LEFT_VELOCITY_SIZE;
}


int ccr::SerialDataPicker::parseRightEncoderCounts(unsigned char * buffer, int index)
{
	// Right encoder counts
	uint16_t right_encoder_counts = buffer2unsigned_int(buffer, index);

	//printf("Right Encoder: %d\n", rightEncoderCounts);

	if(encoder_counts_[RIGHT] == -1 || (int)(right_encoder_counts) == last_encoder_counts_[RIGHT])	// First time, we need 2 to make it work!
	{
		encoder_counts_[RIGHT] = 0;
	}
	else
	{
		encoder_counts_[RIGHT] = (int)(right_encoder_counts - last_encoder_counts_[RIGHT]);
		
 		if(encoder_counts_[RIGHT] > max_encoder_counts_/10) encoder_counts_[RIGHT] = encoder_counts_[RIGHT] - max_encoder_counts_;

		if(encoder_counts_[RIGHT] < -max_encoder_counts_/10) encoder_counts_[RIGHT] = max_encoder_counts_ + encoder_counts_[RIGHT];

	}
	last_encoder_counts_[RIGHT] = right_encoder_counts;
	if (abs(encoder_counts_[RIGHT]) > encoder_threshold_) {
	  integral_encoder_[RIGHT] += abs(encoder_counts_[RIGHT]);
	} else {
	  encoder_counts_[RIGHT] = 0;
	}
        right_encoders.push_back(encoder_counts_[RIGHT]);
        if (right_encoders.size() > SAMPLE_COUNT)
        {
            right_encoders.erase(right_encoders.begin()); 
        }
	
	ROS_DEBUG("Right Encoder: %d encoder[RIGHT]=%d last=%d\n", right_encoder_counts, encoder_counts_[ RIGHT ], last_encoder_counts_[RIGHT] );
	return SENSOR_RIGHT_ENCODER_SIZE;
}

int ccr::SerialDataPicker::parseLeftEncoderCounts(unsigned char * buffer, int index)
{
	// Left encoder counts
	uint16_t left_encoder_counts = buffer2unsigned_int(buffer, index);

	//printf("Left Encoder: %d\n", leftEncoderCounts);
	if(encoder_counts_[LEFT] == -1 || (int)left_encoder_counts == last_encoder_counts_[LEFT])	// First time, we need 2 to make it work!
	{
		encoder_counts_[LEFT] = 0;
	}
	else
	{
		encoder_counts_[LEFT] = (int)(left_encoder_counts - last_encoder_counts_[LEFT]);
	ROS_DEBUG("2Left Encoder: %d encoder[LEFT]=%d last=%d diff=%d\n", left_encoder_counts, encoder_counts_[ LEFT ] ,last_encoder_counts_[LEFT], left_encoder_counts - last_encoder_counts_[LEFT]);
		
 		if(encoder_counts_[LEFT] > max_encoder_counts_/10) encoder_counts_[LEFT] = encoder_counts_[LEFT] - max_encoder_counts_;

 		if(encoder_counts_[LEFT] < -max_encoder_counts_/10) encoder_counts_[LEFT] = max_encoder_counts_ + encoder_counts_[LEFT];

	}
	last_encoder_counts_[LEFT] = left_encoder_counts;
	if (abs(encoder_counts_[LEFT]) > encoder_threshold_) {
	  integral_encoder_[LEFT] += abs(encoder_counts_[LEFT]);
	} else {
	  encoder_counts_[LEFT] = 0;
	}
        left_encoders.push_back(encoder_counts_[LEFT]);
        if (left_encoders.size() > SAMPLE_COUNT)
        {
            left_encoders.erase(left_encoders.begin()); 
        }
	
	ROS_DEBUG("Left Encoder: %d encoder[LEFT]=%d last=%d\n", left_encoder_counts, encoder_counts_[ LEFT ] ,last_encoder_counts_[LEFT]);
	return SENSOR_LEFT_ENCODER_SIZE;
}
	
int ccr::SerialDataPicker::parseLeftMotorCurrent(unsigned char * buffer, int index)
{
	// Left motor current
	this->motor_current_[LEFT] = buffer2signed_int(buffer, index);
	
	return SENSOR_LEFT_MOTOR_CURRENT_SIZE;
}

int ccr::SerialDataPicker::parseRightMotorCurrent(unsigned char * buffer, int index)
{
	// Left motor current
	this->motor_current_[RIGHT] = buffer2signed_int(buffer, index);
	
	return SENSOR_RIGHT_MOTOR_CURRENT_SIZE;
}

int ccr::SerialDataPicker::parseCenterCliffSignal(unsigned char * buffer, int index)
{
	// Cliff signals
	this->cliff_signal_[CENTER] = buffer2unsigned_int(buffer, index);
	
	return SENSOR_CLIFF_CENTER_SIGNAL_SIZE;
}

int ccr::SerialDataPicker::parseCenterCliff(unsigned char * buffer, int index)
{
	// Cliffs
	this->cliff_[CENTER] = buffer[index] & 0x01;
	
	return SENSOR_CLIFF_CENTER_SIZE;
}

int ccr::SerialDataPicker::parseOpticalRangingLeft(unsigned char * buffer, int index)
{
	// Infrared range
        this->optical_range_[LEFT] = buffer2unsigned_int(buffer, index);
	
	return SENSOR_OPTICAL_RANGING_LEFT_SIZE;
}

int ccr::SerialDataPicker::parseOpticalRangingRight(unsigned char * buffer, int index)
{
	// Infrared range
        this->optical_range_[RIGHT] = buffer2unsigned_int(buffer, index);
	
	return SENSOR_OPTICAL_RANGING_RIGHT_SIZE;
}

int ccr::SerialDataPicker::parseUltraSonic(unsigned char * buffer, int index)
{
	// ultra range
        this->ultra_sonic_range_[LEFT] = buffer2unsigned_int(buffer, index);
        this->ultra_sonic_range_[RIGHT] = buffer2unsigned_int(buffer, index + 2);
        this->ultra_sonic_range_[FRONT] = buffer2unsigned_int(buffer, index + 4);
        this->ultra_sonic_range_[BACK] = buffer2unsigned_int(buffer, index + 6);
	
	return SENSOR_ULTRASONIC_SIZE;
}

int ccr::SerialDataPicker::parseSlipDetect(unsigned char * buffer, int index)
{
	// slip detect
        this->slip_ = buffer[index] & 0x01;
	
	return SENSOR_SLIP_DETECT_SIZE;
}

int ccr::SerialDataPicker::parseBrushes(unsigned char * buffer, int index)
{
        this->motor_current_[MAIN_BRUSH] = buffer2signed_int(buffer, index);
        this->encoder_counts_[MAIN_BRUSH] = buffer2signed_int(buffer, index + 2);
	
	return SENSOR_BRUSHES_SIZE;
}

int ccr::SerialDataPicker::parseSaction(unsigned char * buffer, int index)
{
        this->motor_current_[SACTION] = buffer2signed_int(buffer, index);
        this->encoder_counts_[SACTION] = buffer2signed_int(buffer, index + 2);
	
	return SENSOR_SACTION_SIZE;
}

int ccr::SerialDataPicker::parseDirtDetect(unsigned char * buffer, int index)
{
        this->dirt_detect_[DETECTION_LOW] = buffer[index];
        this->dirt_detect_[DETECTION_HIGH] = buffer[index + 1];
	
	return SENSOR_DIRT_DETECTION_SIZE;
}

int ccr::SerialDataPicker::parseDustboxDetect(unsigned char * buffer, int index)
{
	unsigned char byte = buffer[index];

        this->dustbox_ = byte;
	
	return SENSOR_DUSTBOX_DETECTION_SIZE;
}

int ccr::SerialDataPicker::parseDustAmount(unsigned char * buffer, int index)
{
	unsigned char byte = buffer[index];

        this->dust_amount_ = byte;
	
	return SENSOR_DUST_AMOUNT_SIZE;
}

int ccr::SerialDataPicker::parseLedBoard(unsigned char * buffer, int index)
{
	memcpy(this->led_board_event_, buffer+index, SENSOR_LED_BOARD_EVENT_SIZE);
	
	return SENSOR_LED_BOARD_EVENT_SIZE;
}

int ccr::SerialDataPicker::parseIrChar(unsigned char * buffer, int index)
{
    for (int i = 0; i <= IR_CENTER; i++)
    {
	    memcpy(this->ir_char_[i]
			, buffer+index+(i*SENSOR_IR_CHAR_EACH_SIZE)
			, SENSOR_IR_CHAR_EACH_SIZE);
	}
	return SENSOR_IR_CHAR_EACH_SIZE * (IR_CENTER + 1);
}


int ccr::SerialDataPicker::buffer2signed_int(unsigned char * buffer, int index)
{
	int16_t signed_int;
	
	memcpy(&signed_int, buffer+index, 2);
	signed_int = ntohs(signed_int);
	
	return (int)signed_int;
}

int ccr::SerialDataPicker::buffer2unsigned_int(unsigned char * buffer, int index)
{
	uint16_t unsigned_int;

	memcpy(&unsigned_int, buffer+index, 2);
	unsigned_int = ntohs(unsigned_int);
	
	return (int)unsigned_int;
}

// *****************************************************************************
// Calculate CCR odometry
void ccr::SerialDataPicker::calculateOdometry()
{
/*	   
    const double left_pose = (encoder_counts_[LEFT])*2*M_PI/300.0;
    const double right_pose = (encoder_counts_[RIGHT])*2*M_PI/300.0;
*/
    double ave_left = 0;
    for(auto i = left_encoders.begin(); i != left_encoders.end(); i++) 
    {
        ave_left += *i;
	//printf("---left[%f]---\n", *i);
    }
    ave_left /= left_encoders.size();

    double ave_right = 0;//std::accumulate(right_encoders.begin(), right_encoders.end(), 0) / right_encoders.size();
    for(auto i = right_encoders.begin(); i != right_encoders.end(); i++) 
    {
        ave_right += *i;
	//printf("---right[%f]---\n", *i);
    }
    ave_right /= right_encoders.size();

    const double left_pose = ave_left*2*M_PI/max_cycle_encoder_counts_;
    const double right_pose = ave_right*2*M_PI/max_cycle_encoder_counts_;

    //std::cout << left_pose << "\t" << right_pose << std::endl;

    this->new_odometry_.update(left_pose, right_pose, ros::Time::now());
   
    this->odometry_yaw_ = this->new_odometry_.getHeading();        // rad
    this->odometry_x_ = this->new_odometry_.getX();                // m
    this->odometry_y_ = this->new_odometry_.getY();                // m

	ROS_DEBUG("odometry_x_: %lf\n", this->odometry_x_ );
}


// *****************************************************************************
// Reset CCR odometry
void ccr::SerialDataPicker::resetOdometry()
{   
    this->new_odometry_.resetOdometry();
    this->odometry_x_ = this->new_odometry_.getX();
    this->odometry_y_ = this->new_odometry_.getY();
    this->odometry_yaw_ = this->new_odometry_.getHeading();
    this->right_encoders.clear();
    this->left_encoders.clear();
}


// *****************************************************************************
// Set CCR odometry
void ccr::SerialDataPicker::setOdometry(double new_x, double new_y, double new_yaw)
{
	this->odometry_x_ = new_x;
	this->odometry_y_ = new_y;
	this->odometry_yaw_ = new_yaw;
}

// *****************************************************************************
// Export wheel rotation to syslog
void ccr::SerialDataPicker::logger(const char* p_Fmt, ...)
{
  va_list  arg;
  openlog(NULL, 0, LOG_LOCAL4);
  va_start(arg, p_Fmt);
  vsyslog(LOG_NOTICE, p_Fmt, arg);
  va_end(arg);
  closelog();
}

void ccr::SerialDataPicker::exportWheelRotation()
{
  float    total;

  total = ((integral_encoder_[LEFT] + integral_encoder_[RIGHT]) / 2.0 / (float)max_cycle_encoder_counts_)
    * M_PI * wheel_diameter_;
  ROS_DEBUG("Integral Wheel count: %f", total);
  logger("Integral Wheel count: %f", total);
}

// EOF
