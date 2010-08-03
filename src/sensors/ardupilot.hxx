//
// FILE: ardupilot.hxx
// DESCRIPTION: interact with ardupilot based servo subsystem board
//

#ifndef _UGEAR_ARDUPILOT_HXX
#define _UGEAR_ARDUPILOT_HXX


#include "include/globaldefs.h"

#include "props/props.hxx"


// function prototypes
bool ardupilot_init( SGPropertyNode *config );
bool ardupilot_update();
void ardupilot_close();

bool ardupilot_airdata_init( string rootname );
bool ardupilot_airdata_update();

bool ardupilot_pilot_init( string rootname );
bool ardupilot_pilot_update();


#endif // _UGEAR_ARDUPILOT_HXX
