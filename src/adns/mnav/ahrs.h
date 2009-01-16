/*******************************************************************************
 * FILE: ahrs.h
 * DESCRIPTION: attitude heading reference system providing the attitude of
 *   	       the vehicle using an extended Kalman filter
 ******************************************************************************/

#ifndef _UGEAR_AHRS_H
#define _UGEAR_AHRS_H


extern struct imu imupacket;
extern double xs[7];

void ahrs_init();
void ahrs_update();
void ahrs_close();


#endif // _UGEAR_AHRS_H
