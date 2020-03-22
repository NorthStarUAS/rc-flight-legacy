//
// FILE: act_fgfs.h
// DESCRIPTION: send actuator commands to FlightGear
//

#pragma once

#include <pyprops.h>

#include "include/globaldefs.h"


// function prototypes
bool fgfs_act_init( pyPropertyNode *config );
bool fgfs_act_update();
void fgfs_act_close();
