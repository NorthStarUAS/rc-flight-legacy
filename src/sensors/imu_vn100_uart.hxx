/**
 * \file: imu_vn100_uart.hxx
 *
 * VectorNav VN100_UART (UART) driver
 *
 * Copyright (C) 2012 - Curtis L. Olson colson@atiak.com
 *
 */

#ifndef _UGEAR_IMU_VN100_UART_HXX
#define _UGEAR_IMU_VN100_UART_HXX


#include <string>

#include "include/globaldefs.h"
#include "props/props.hxx"

using std::string;


void imu_vn100_uart_init( string rootname, SGPropertyNode *config );
bool imu_vn100_uart_get();
void imu_vn100_uart_close();


#endif // _UGEAR_IMU_VN100_UART_HXX
