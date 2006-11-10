/*******************************************************************************
 * FILE: ahrs.h
 * DESCRIPTION: attitude heading reference system providing the attitude of
 *   	       the vehicle using an extended Kalman filter
 ******************************************************************************/

#ifndef _UGEAR_AHRS_H
#define _UGEAR_AHRS_H


extern double xs[7];

void *ahrs_thread(void *thread_id);


#endif // _UGEAR_AHRS_H
