//
// FILE: act_goldy2.hxx
// DESCRIPTION: send actuator commands to Goldy2
//

#ifndef _AURA_ACT_GOLDY2_HXX
#define _AURA_ACT_GOLDY2_HXX


#include "python/pyprops.hxx"

#include "include/globaldefs.h"


// function prototypes
bool goldy2_act_init( string output_path, pyPropertyNode *config );
bool goldy2_act_update();
void goldy2_act_close();


#endif // _AURA_ACT_GOLDY2_HXX
