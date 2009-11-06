//
// FILE: mnav.h
// DESCRIPTION: aquire, validate and decode the IMU, GPS, and Servo data
//              inbound from the MNAV
//

#ifndef _UGEAR_MNAV_H
#define _UGEAR_MNAV_H


#include "globaldefs.h"

#include "props/props.hxx"


// global definitions
extern bool autopilot_active;
extern bool autopilot_reinit;

struct gps {
   double time;
   double lat,lon,alt;          /* gps position                */
   double vn,ve,vd;             /* gps velocity                */
   double date;                 /* unix seconds from gps       */
   uint64_t status;		/* data status flag            */
};

// function prototypes
void mnav_imu_init( string rootname, SGPropertyNode *config );
void mnav_gps_init( string rootname );

bool mnav_read();

void mnav_start_nonblock_read(); // call before first nonblock read
bool mnav_read_nonblock();

void mnav_close();

bool mnav_get_imu();
bool mnav_get_gps();
bool mnav_get_airdata( struct imu *data );

void mnav_imu_update();
void mnav_gps_update();
void mnav_airdata_update();
void mnav_manual_override_check();

void mnav_send_servo_cmd( struct servo *servo_out );
void mnav_send_short_servo_cmd( struct servo *servo_out );


#endif // _UGEAR_MNAV_H
