//
// FILE: ardusensor.hxx
// DESCRIPTION: interact with ardupilot based servo subsystem board
//

#ifndef _UGEAR_ARDUSENSOR_HXX
#define _UGEAR_ARDUSENSOR_HXX


#include "include/globaldefs.h"

#include "props/props.hxx"


// function prototypes
bool ardusensor_init( SGPropertyNode *config );
bool ardusensor_update();
void ardusensor_close();

bool ardusensor_airdata_init( string rootname );
bool ardusensor_airdata_update();

bool ardusensor_pilot_init( string rootname );
bool ardusensor_pilot_update();


#endif // _UGEAR_ARDUSENSOR_HXX
