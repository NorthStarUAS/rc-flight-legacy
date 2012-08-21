/**
 * \file: imu_vn100_spi.hxx
 *
 * VectorNav VN100_SPI (SPI) driver
 *
 * Copyright (C) 2012 - Curtis L. Olson colson@atiak.com
 *
 */

#ifndef _UGEAR_IMU_VN100_SPI_HXX
#define _UGEAR_IMU_VN100_SPI_HXX


#include <string>

#include "include/globaldefs.h"
#include "props/props.hxx"

using std::string;


void imu_vn100_spi_init( string rootname, SGPropertyNode *config );
bool imu_vn100_spi_get();
void imu_vn100_spi_close();


#endif // _UGEAR_IMU_VN100_SPI_HXX
