/**
 * Copyright(C) 2017 Panasonic Corporation. All Right reserved.
 * 
 * \file        Ccr_Constants.h
 * \brief       Constant parameter definition.
 * \date        2017/02/16  create a new
 *
 * \note        
 *
 */

// Topics
//! prefix for sensor data publish.
#define TOPIC_EVENT_PREFIX "mobile_base/event/"
//! prefix for user's request.
#define TOPIC_COMMAND_PREFIX "mobile_base/command/"

// Rulo state (for topic "mode")
//! move as user's request. if bumper/emergency stop is detected,
// Rulo stop until getting topic different "mode".
#define CCR_STATE_NORMAL "normal"
//! stoppig because of bumper/emergency stop detection.
#define CCR_STATE_FORCE_STOP "force_stop"
//! Rulo doesn't stop regardless of sensor data
// except for emergency stop button.
#define CCR_STATE_MANUAL "manual"

// Positions
#define LEFT				0
#define RIGHT				1
#define FRONT_LEFT			2
#define FRONT_RIGHT			3
#define CENTER   			4

#define FRONT				2
#define BACK				3

#define MAIN_BRUSH			2
#define SACTION				3

#define DETECTION_LOW		0
#define DETECTION_HIGH		1

#define IR_OMNI				0
#define IR_LEFT				1
#define IR_RIGHT				2
#define IR_CENTER				3

// Dimensions
#define CCR_AXLE_LENGTH			0.461
#define CCR_WHEEL_DIAMETER       0.16

// Limits
#define CCR_MAX_LIN_VEL_MM_S		500
#define CCR_MAX_ANG_VEL_RAD_S	2  
#define CCR_MAX_RADIUS_MM		2000

// max encoder counts
#define CCR_MAX_ENCODER_COUNTS	65535
#define CCR_CYCLE_ENCODER_COUNTS	300

// Logging rate of encoder counts
#define CCR_ENCODER_LOGGING_RATE       (1.0/2.0)
#define CCR_ENCODER_LOGGING_THRESHOLD  (0)
