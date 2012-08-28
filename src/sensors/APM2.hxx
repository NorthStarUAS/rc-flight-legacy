//
// FILE: APM2.hxx
// DESCRIPTION: interact with APM2 converted to a sensor head
//

#ifndef _UGEAR_APM2_HXX
#define _UGEAR_APM2_HXX


#include "include/globaldefs.h"

#include "props/props.hxx"


// function prototypes

bool APM2_update();
void APM2_close();
bool APM2_request_baud( uint32_t baud );

bool APM2_imu_init( string rootname, SGPropertyNode *config );
bool APM2_imu_update();
void APM2_imu_close();

bool APM2_gps_init( string rootname, SGPropertyNode *config  );
bool APM2_gps_update();
void APM2_gps_close();

bool APM2_airdata_init( string rootname );
bool APM2_airdata_update();
void APM2_airdata_close();

bool APM2_pilot_init( string rootname );
bool APM2_pilot_update();
void APM2_pilot_close();

bool APM2_act_init( SGPropertyNode *config );
bool APM2_act_update();
void APM2_act_close();


#endif // _UGEAR_APM2_HXX
