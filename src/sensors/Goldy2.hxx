//
// FILE: imu_fgfs.hxx
// DESCRIPTION: aquire live sensor data from a live running instance
// of Flightgear
//

#ifndef _AURA_GOLDY2_HXX
#define _AURA_GOLDY2_HXX


#include "include/globaldefs.h"

#include "props/props.hxx"


// function prototypes
bool goldy2_open();
bool goldy2_update();
bool goldy2_close();

bool goldy2_imu_init( string rootname, SGPropertyNode *config );
bool goldy2_imu_update();
void goldy2_imu_close();

bool goldy2_gps_init( string rootname, SGPropertyNode *config  );
bool goldy2_gps_update();
void goldy2_gps_close();

bool goldy2_airdata_init( string rootname, SGPropertyNode *config );
bool goldy2_airdata_update();
void goldy2_airdata_close();

bool goldy2_pilot_init( string rootname, SGPropertyNode *config );
bool goldy2_pilot_update();
void goldy2_pilot_close();

#endif // _AURA_GOLDY2_HXX
