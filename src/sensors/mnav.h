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
extern char *cnt_status;

#define MAX_MNAV_DEV 64
extern char mnav_dev[MAX_MNAV_DEV];

enum mnav_result_t {
    NoValidData,		/* data didn't parse or checksum correctly */
    IMUValid,			/* Valid IMU data was loaded into the
				   global structure */
    GPSValid,			/* Valid GPS data was loaded into the
				   global structure */
    IMUGPSValid,		/* Valid IMU and GPS data was loaded
				   into the global structure */
};


// function prototypes
void mnav_init();
mnav_result_t mnav_read();
void mnav_imu_update();
void mnav_gps_update();
void mnav_servo_update();
void mnav_close();

void send_servo_cmd();
void send_short_servo_cmd();


#endif // _UGEAR_MNAV_H
