/*! \file nav_interface.h
 *	\brief Navigation filter interface header
 *
 *	\details This file declares the standard function prototypes for interfacing with the navigation filter.
 *	All navigation filters must include this file and instantiate the init_nav(), get_nav(), and close_nav() functions.
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: nav_interface.h 757 2012-01-04 21:57:48Z murch $
 */

#ifndef NAV_INTERFACE_H_
#define NAV_INTERFACE_H_

#include "globaldefs.h"

#ifdef __cplusplus
extern "C" {
#endif

/// Standard function to initialize the navigation filter.
/*!
 * \sa get_nav(), close_nav()
 * \ingroup nav_fcns
*/
void init_nav( struct imu *imuData_ptr, // pointer to imu structure
	       struct gps *gpsData_ptr, // pointer to gps structure
	       struct nav *navData_ptr  // pointer to navData structure
	       );

/// Standard function to call the navigation filter.
/*!
 * \sa init_nav(), close_nav()
 * \ingroup nav_fcns
*/
void get_nav( struct imu *imuData_ptr, // pointer to imu structure
	      struct gps *gpsData_ptr, // pointer to gps structure
	      struct nav *navData_ptr  // pointer to navData structure
	      );

/// Standard function to close the navigation filter.
/*!
 * No input or return parameters
 * \sa get_nav(), init_nav()
 * \ingroup nav_fcns
*/
void close_nav(void);


#ifdef __cplusplus
}
#endif

#endif
