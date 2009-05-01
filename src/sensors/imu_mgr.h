/**
 * \file: imu_mgr.h
 *
 * Front end management interface for reading IMU data.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@gmail.com
 *
 * $Id: imu_mgr.h,v 1.3 2009/05/01 02:04:17 curt Exp $
 */


#ifndef _UGEAR_IMU_MGR_H
#define _UGEAR_IMU_MGR_H


void IMU_init();
bool IMU_update();
void IMU_close();


#endif // _UGEAR_IMU_MGR_H
