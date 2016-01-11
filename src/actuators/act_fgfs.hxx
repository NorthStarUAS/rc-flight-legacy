//
// FILE: act_fgfs.hxx
// DESCRIPTION: send actuator commands to FlightGear
//

#ifndef _AURA_ACT_FGFS_HXX
#define _AURA_ACT_FGFS_HXX


#include "python/pyprops.hxx"

#include "include/globaldefs.h"


// function prototypes
bool fgfs_act_init( pyPropertyNode *config );
bool fgfs_act_update();
void fgfs_act_close();


#endif // _AURA_ACT_FGFS_HXX
