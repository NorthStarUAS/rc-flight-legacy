/**
 * \file: IMU.h
 *
 * Front end management interface for reading IMU data.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@gmail.com
 *
 * $Id: IMU.h,v 1.1 2009/01/17 20:28:01 curt Exp $
 */


#ifndef _UGEAR_IMU_H
#define _UGEAR_IMU_H


extern struct imu imupacket;

enum imu_source_t {
    imuNone,
    imuMNAV
};

void IMU_init();
void IMU_update();
void IMU_close();


#endif // _UGEAR_IMU_H
