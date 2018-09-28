/**
 * \file: imu_vn100_uart.hxx
 *
 * VectorNav VN100_UART (UART) driver
 *
 * Copyright (C) 2012 - Curtis L. Olson - curtolson@flightgear.org
 *
 */

#pragma once

#include <pyprops.hxx>

#include <string>
using std::string;

#include "include/globaldefs.h"

void imu_vn100_uart_init( string output_path, pyPropertyNode *config );
bool imu_vn100_uart_get();
void imu_vn100_uart_close();
