/**
 * \file: imu_mgr.h
 *
 * Front end management interface for reading IMU data.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@gmail.com
 *
 * $Id: imu_mgr.h,v 1.2 2009/04/30 14:39:39 curt Exp $
 */


#ifndef _UGEAR_IMU_MGR_H
#define _UGEAR_IMU_MGR_H


//extern struct imu imupacket;

enum imu_source_t {
    imuNone,
    imuMNAV,
    imuUGFile
};

void IMU_init();
bool IMU_update();
void IMU_close();


#endif // _UGEAR_IMU_MGR_H
