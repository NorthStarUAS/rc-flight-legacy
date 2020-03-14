//
// FILE: Aura4.h
// DESCRIPTION: interact with Aura4 (Teensy/Pika) sensor head
//

#pragma once

#include <pyprops.h>

#include "drivers/driver.h"

#include "include/globaldefs.h" /* fixme, get rid of? */

class Aura4_t: public driver_t {
public:
    Aura4_t();
    ~Aura4_t();
    void init( pyPropertyNode *config );
    void read();
    void wrte();
    void close();
};

// function prototypes

double Aura4_update();
void Aura4_close();
bool Aura4_request_baud( uint32_t baud );

bool Aura4_imu_init( string output_path, pyPropertyNode *config );
bool Aura4_imu_update();
void Aura4_imu_close();

bool Aura4_gps_init( string output_path, pyPropertyNode *config  );
bool Aura4_gps_update();
void Aura4_gps_close();

bool Aura4_airdata_init( string output_path );
bool Aura4_airdata_update();
// force an airspeed zero calibration (ideally with the aircraft on
// the ground with the pitot tube perpendicular to the prevailing
// wind.)
void Aura4_airdata_zero_airspeed();
void Aura4_airdata_close();

bool Aura4_pilot_init( string output_path, pyPropertyNode *config );
bool Aura4_pilot_update();
void Aura4_pilot_close();

bool Aura4_act_init( pyPropertyNode *config );
bool Aura4_act_update();
void Aura4_act_close();
extern bool Aura4_actuator_configured;
