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

bool mnav_read();

void mnav_start_nonblock_read(); // call before first nonblock read
bool mnav_read_nonblock();

void mnav_close();

bool mnav_get_imu( struct imu *data );
bool mnav_get_gps( struct gps *data );
bool mnav_get_press();

void mnav_imu_update();
void mnav_gps_update();
void mnav_press_update();
void mnav_manual_override_check();

void mnav_send_servo_cmd( struct servo *servo_out );
void mnav_send_short_servo_cmd( struct servo *servo_out );


#endif // _UGEAR_MNAV_H
