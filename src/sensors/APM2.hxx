//
// FILE: APM2.hxx
// DESCRIPTION: interact with APM2 converted to a sensor head
//

#ifndef _AURA_APM2_HXX
#define _AURA_APM2_HXX


#include "python/pyprops.hxx"

#include "include/globaldefs.h"


// function prototypes

bool APM2_update();
void APM2_close();
bool APM2_request_baud( uint32_t baud );

bool APM2_imu_init( string rootname, pyPropertyNode *config );
bool APM2_imu_update();
void APM2_imu_close();

bool APM2_gps_init( string rootname, pyPropertyNode *config  );
bool APM2_gps_update();
void APM2_gps_close();

bool APM2_airdata_init( string rootname );
bool APM2_airdata_update();
// force an airspeed zero calibration (ideally with the aircraft on
// the ground with the pitot tube perpendicular to the prevailing
// wind.)
void APM2_airdata_zero_airspeed();
void APM2_airdata_close();

bool APM2_pilot_init( string rootname );
bool APM2_pilot_update();
void APM2_pilot_close();

bool APM2_act_init( pyPropertyNode *config );
bool APM2_act_update();
void APM2_act_close();


#endif // _AURA_APM2_HXX
