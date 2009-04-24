/*******************************************************************************
 * FILE: ahrs.h
 * DESCRIPTION: attitude heading reference system providing the attitude of
 *   	       the vehicle using an extended Kalman filter
 ******************************************************************************/

#ifndef _UGEAR_AHRS_H
#define _UGEAR_AHRS_H


extern double xs[7];

void mnav_ahrs_init();
void mnav_ahrs_update();
void mnav_ahrs_close();


#endif // _UGEAR_AHRS_H
