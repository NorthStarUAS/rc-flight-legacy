//
// FILE: act_goldy2.hxx
// DESCRIPTION: send actuator commands to Goldy2
//

#pragma once

#include "python/pyprops.hxx"

#include "include/globaldefs.h"

// function prototypes
bool goldy2_act_init( pyPropertyNode *config );
bool goldy2_act_update();
void goldy2_act_close();
