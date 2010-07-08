/**
 * \file: sf_6DOFv4.h
 *
 * Sparkfun 6DOF v4 driver
 *
 * Copyright Curt Olson curtolson@gmail.com
 *
 * $Id: gpsd.h,v 1.2 2009/05/01 02:04:17 curt Exp $
 */

#ifndef _UGEAR_IMU_VN100_H
#define _UGEAR_IMU_VN100_H

#include <string>

#include "include/globaldefs.h"
#include "props/props.hxx"

using std::string;


void imu_vn100_init( string rootname, SGPropertyNode *config );
bool imu_vn100_get();
void imu_vn100_close();


#endif // _UGEAR_IMU_VN100_H
