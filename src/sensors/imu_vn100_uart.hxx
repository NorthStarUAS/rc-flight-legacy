/**
 * \file: imu_vn100_uart.hxx
 *
 * VectorNav VN100_UART (UART) driver
 *
 * Copyright (C) 2012 - Curtis L. Olson - curtolson@flightgear.org
 *
 */

#ifndef _AURA_IMU_VN100_UART_HXX
#define _AURA_IMU_VN100_UART_HXX


#include <string>

#include "include/globaldefs.h"
#include "python/pyprops.hxx"

using std::string;


void imu_vn100_uart_init( string rootname, SGPropertyNode *config );
bool imu_vn100_uart_get();
void imu_vn100_uart_close();


#endif // _AURA_IMU_VN100_UART_HXX
