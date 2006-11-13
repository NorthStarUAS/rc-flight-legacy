//
// FILE: imugps.h
// DESCRIPTION: aquire, validate and decode the IMU, GPS, and Servo data
//              inbound from the uNAV
//

#ifndef _UGEAR_IMUGPS_H
#define _UGEAR_IMUGPS_H


#include "globaldefs.h"


// global definitions
extern struct servo servopacket;
extern bool autopilot_enable;
extern bool control_init;
extern char *cnt_status;


// function prototypes
void *imugps_acq(void *thread_id);
void send_servo_cmd(uint16_t cnt_cmd[9]);


#endif // _UGEAR_IMUGPS_H
