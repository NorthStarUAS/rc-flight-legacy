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


#include "python/pyprops.hxx"

#include <string>
using std::string;

#include "include/globaldefs.h"


void imu_vn100_uart_init( pyPropertyNode *base, pyPropertyNode *config );
bool imu_vn100_uart_get();
void imu_vn100_uart_close();


#endif // _AURA_IMU_VN100_UART_HXX
