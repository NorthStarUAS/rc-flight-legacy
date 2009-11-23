//
// FILE: imu_fgfs.hxx
// DESCRIPTION: aquire live sensor data from a live running instance
// of Flightgear
//

#ifndef _UGEAR_IMU_FGFS_HXX
#define _UGEAR_IMU_FGFS_HXX


#include "include/globaldefs.h"

#include "props/props.hxx"


// function prototypes
bool fgfs_imu_init( string rootname, SGPropertyNode *config );
bool fgfs_imu_update();
void fgfs_imu_close();

//bool fgfs_imu_get();


#endif // _UGEAR_IMU_FGFS_HXX
