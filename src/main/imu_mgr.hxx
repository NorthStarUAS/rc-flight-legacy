//
// imu_mgr.hxx - front end IMU sensor management interface
//
// Written by Curtis Olson, curtolson <at> gmail <dot> com.  Spring 2009.
// This code is released into the public domain.
// 


#ifndef _UGEAR_IMU_MGR_HXX
#define _UGEAR_IMU_MGR_HXX


void IMU_init();
bool IMU_update();
void IMU_close();


#endif // _UGEAR_IMU_MGR_HXX
