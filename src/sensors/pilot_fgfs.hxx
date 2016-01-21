//
// FILE: pilot_fgfs.hxx
// DESCRIPTION: aquire live sensor data from a live running instance
// of Flightgear
//

#ifndef _AURA_PILOT_FGFS_HXX
#define _AURA_PILOT_FGFS_HXX


#include "python/pyprops.hxx"

#include "include/globaldefs.h"


// function prototypes
bool fgfs_pilot_init( pyPropertyNode *base, pyPropertyNode *config );
bool fgfs_pilot_update();
void fgfs_pilot_close();


#endif // _AURA_PILOT_FGFS_HXX
