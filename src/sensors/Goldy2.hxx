//
// FILE: Goldy2.hxx
// DESCRIPTION: aquire sensor data from Goldy2 FMU (via ethernet)
//

#ifndef _AURA_GOLDY2_HXX
#define _AURA_GOLDY2_HXX


#include "python/pyprops.hxx"

#include "include/globaldefs.h"


// function prototypes
bool goldy2_open();
bool goldy2_update();
bool goldy2_close();

bool goldy2_imu_init( string output_path, pyPropertyNode *config );
bool goldy2_imu_update();
void goldy2_imu_close();

bool goldy2_gps_init( string output_path  );
bool goldy2_gps_update();
void goldy2_gps_close();

bool goldy2_airdata_init( string output_path );
bool goldy2_airdata_update();
void goldy2_airdata_close();

bool goldy2_pilot_init( string output_path, pyPropertyNode *config );
bool goldy2_pilot_update();
void goldy2_pilot_close();

#endif // _AURA_GOLDY2_HXX
