//
// FILE: act_fgfs.hxx
// DESCRIPTION: send actuator commands to FlightGear
//

#ifndef _UGEAR_ACT_FGFS_HXX
#define _UGEAR_ACT_FGFS_HXX


#include "include/globaldefs.h"

#include "props/props.hxx"


// function prototypes
bool fgfs_act_init( SGPropertyNode *config );
bool fgfs_act_update();
void fgfs_act_close();


#endif // _UGEAR_ACT_FGFS_HXX
