/**
 * \file: IMU.h
 *
 * Front end management interface for reading IMU data.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@gmail.com
 *
 * $Id: IMU.h,v 1.3 2009/04/17 18:10:03 curt Exp $
 */


#ifndef _UGEAR_IMU_H
#define _UGEAR_IMU_H


extern struct imu imupacket;

enum imu_source_t {
    imuNone,
    imuMNAV,
    imuUGFile
};

void IMU_init();
bool IMU_update();
void IMU_close();


#endif // _UGEAR_IMU_H
