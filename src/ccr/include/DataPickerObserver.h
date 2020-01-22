/**
 * Copyright(C) 2017 Panasonic Corporation. All Right reserved.
 * 
 * \file        DataPickerObserver.h
 * \brief       Observer class for sensor data.
 * \date        2017/01/25  create a new
 *
 * \note        
 *
 */
#ifndef __DATA_PICKER_OBSERVER_H
#define __DATA_PICKER_OBSERVER_H

#include "diversion/odometry.h"

namespace ccr
{
	/*! \class DataPickerObserver
	 *  \brief Observer class for sensor data.
	 */
	class DataPickerObserver
	{
		public:
	
        /***********************************************************************/
        /* functions */
        /***********************************************************************/
		//! Constructor
		DataPickerObserver() {};
		//! Destructor
		virtual ~DataPickerObserver() {};
	
		//! Notify that new sensor data is available.
		virtual void onUpdate() = 0;
	};

}

#endif // __DATA_PICKER_OBSERVER_H

// EOF
