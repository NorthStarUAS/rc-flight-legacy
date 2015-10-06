//
// imu_mgr.hxx - front end IMU sensor management interface
//
// Written by Curtis Olson, curtolson <at> gmail <dot> com.  Spring 2009.
// This code is released into the public domain.
// 


#ifndef _AURA_IMU_MGR_HXX
#define _AURA_IMU_MGR_HXX


void IMU_init();
bool IMU_update();
void IMU_close();

// return imu data age in seconds
double IMU_age();

#endif // _AURA_IMU_MGR_HXX
