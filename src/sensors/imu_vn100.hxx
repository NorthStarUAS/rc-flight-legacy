/**
 * \file: imu_vn100.hxx
 *
 * VectorNav VN100 (UART) driver
 *
 * Copyright (C) 2012 - Curtis L. Olson colson@atiak.com
 *
 */

#ifndef _UGEAR_IMU_VN100_HXX
#define _UGEAR_IMU_VN100_HXX


#include <string>

#include "include/globaldefs.h"
#include "props/props.hxx"

using std::string;


void imu_vn100_init( string rootname, SGPropertyNode *config );
bool imu_vn100_get();
void imu_vn100_close();


#endif // _UGEAR_IMU_VN100_HXX
