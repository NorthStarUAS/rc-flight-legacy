//
// FILE: ugfile.h
// DESCRIPTION: aquire saved sensor data from a set of files (rather
// than from a live sensor)
//

#ifndef _UGEAR_FILE_H
#define _UGEAR_FILE_H


#include "globaldefs.h"

#include "props/props.hxx"


// function prototypes
bool ugfile_imu_init( string rootname, SGPropertyNode *config );
bool ugfile_gps_init( string rootname, SGPropertyNode *config );
bool ugfile_read();
void ugfile_close();

bool ugfile_get_imu();
bool ugfile_get_gps();


#endif // _UGEAR_FILE_H
