/**
 * \file: imu_vn100_spi.hxx
 *
 * VectorNav VN100_SPI (SPI) driver
 *
 * Copyright (C) 2012 - Curtis L. Olson - curtolson@flightgear.org
 *
 */

#ifndef _AURA_IMU_VN100_SPI_HXX
#define _AURA_IMU_VN100_SPI_HXX


#include "python/pyprops.hxx"

#include <string>
using std::string;

#include "include/globaldefs.h"


void imu_vn100_spi_init( string output_path, pyPropertyNode *config );
bool imu_vn100_spi_get();
void imu_vn100_spi_close();


#endif // _AURA_IMU_VN100_SPI_HXX
