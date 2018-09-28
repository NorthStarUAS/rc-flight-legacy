//
// FILE: APM2.hxx
// DESCRIPTION: interact with APM2 converted to a sensor head
//

#pragma once

#include <pyprops.hxx>

#include "include/globaldefs.h"

// function prototypes

double APM2_update();
void APM2_close();
bool APM2_request_baud( uint32_t baud );

bool APM2_imu_init( string output_path, pyPropertyNode *config );
bool APM2_imu_update();
void APM2_imu_close();

bool APM2_gps_init( string output_path, pyPropertyNode *config  );
bool APM2_gps_update();
void APM2_gps_close();

bool APM2_airdata_init( string output_path );
bool APM2_airdata_update();
// force an airspeed zero calibration (ideally with the aircraft on
// the ground with the pitot tube perpendicular to the prevailing
// wind.)
void APM2_airdata_zero_airspeed();
void APM2_airdata_close();

bool APM2_pilot_init( string output_path, pyPropertyNode *config );
bool APM2_pilot_update();
void APM2_pilot_close();

bool APM2_act_init( pyPropertyNode *config );
bool APM2_act_update();
void APM2_act_close();
extern bool APM2_actuator_configured;
