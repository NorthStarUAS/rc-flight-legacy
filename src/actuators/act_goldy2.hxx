//
// FILE: act_goldy2.hxx
// DESCRIPTION: send actuator commands to Goldy2
//

#ifndef _AURA_ACT_GOLDY2_HXX
#define _AURA_ACT_GOLDY2_HXX


#include "include/globaldefs.h"

#include "props/props.hxx"


// function prototypes
bool goldy2_act_init( SGPropertyNode *config );
bool goldy2_act_update();
void goldy2_act_close();


#endif // _AURA_ACT_GOLDY2_HXX
