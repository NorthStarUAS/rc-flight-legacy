//
// FILE: pilot_fgfs.hxx
// DESCRIPTION: aquire live sensor data from a live running instance
// of Flightgear
//

#ifndef _AURA_PILOT_FGFS_HXX
#define _AURA_PILOT_FGFS_HXX


#include "include/globaldefs.h"

#include "python/pyprops.hxx"


// function prototypes
bool fgfs_pilot_init( string rootname, SGPropertyNode *config );
bool fgfs_pilot_update();
void fgfs_pilot_close();


#endif // _AURA_PILOT_FGFS_HXX
