//
// FILE: mnav.h
// DESCRIPTION: aquire, validate and decode the IMU, GPS, and Servo data
//              inbound from the MNAV
//

#ifndef _UGEAR_MNAV_H
#define _UGEAR_MNAV_H


#include "globaldefs.h"


// global definitions
extern bool autopilot_active;
extern bool autopilot_reinit;


// function prototypes
void mnav_init();
void mnav_read();
void mnav_close();

bool mnav_get_imu( struct imu *data );
bool mnav_get_gps( struct gps *data );

void mnav_imu_update();
void mnav_gps_update();
void mnav_manual_override_check();

void send_servo_cmd();
void send_short_servo_cmd();


#endif // _UGEAR_MNAV_H
