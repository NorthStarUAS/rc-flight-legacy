//
// FILE: ugfile.h
// DESCRIPTION: aquire saved sensor data from a set of files (rather
// than from a live sensor)
//

#ifndef _AURA_FILE_H
#define _AURA_FILE_H


#include "python/pyprops.hxx"

#include "include/globaldefs.h"


// function prototypes
bool ugfile_imu_init( string output_path, pyPropertyNode *config );
bool ugfile_gps_init( string output_path, pyPropertyNode *config );
bool ugfile_read();
void ugfile_close();

bool ugfile_get_imu();
bool ugfile_get_gps();


#endif // _AURA_FILE_H
