//
// FILE: act_fgfs.hxx
// DESCRIPTION: send actuator commands to FlightGear
//

#pragma once

#include "python/pyprops.hxx"

#include "include/globaldefs.h"


// function prototypes
bool fgfs_act_init( pyPropertyNode *config );
bool fgfs_act_update();
void fgfs_act_close();
