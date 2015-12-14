//
// FILE: act_fgfs.hxx
// DESCRIPTION: send actuator commands to FlightGear
//

#ifndef _AURA_ACT_FGFS_HXX
#define _AURA_ACT_FGFS_HXX


#include "include/globaldefs.h"

#include "props/props.hxx"


// function prototypes
bool fgfs_act_init( SGPropertyNode *config );
bool fgfs_act_update();
void fgfs_act_close();


#endif // _AURA_ACT_FGFS_HXX
