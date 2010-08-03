//
// FILE: pilot_fgfs.hxx
// DESCRIPTION: aquire live sensor data from a live running instance
// of Flightgear
//

#ifndef _UGEAR_PILOT_FGFS_HXX
#define _UGEAR_PILOT_FGFS_HXX


#include "include/globaldefs.h"

#include "props/props.hxx"


// function prototypes
bool fgfs_pilot_init( string rootname, SGPropertyNode *config );
bool fgfs_pilot_update();
void fgfs_pilot_close();


#endif // _UGEAR_PILOT_FGFS_HXX
