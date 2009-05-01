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
bool ugfile_init( string rootname );
bool ugfile_read();
void ugfile_close();

bool ugfile_get_imu( struct imu *data );
bool ugfile_get_gps( struct gps *data );


#endif // _UGEAR_FILE_H
