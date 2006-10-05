/*******************************************************************************
 * FILE: ahrs.h
 * DESCRIPTION: attitude heading reference system providing the attitude of
 *   	       the vehicle using an extended Kalman filter
 ******************************************************************************/

#ifndef _UNAV_AHRS_H
#define _UNAV_AHRS_H


void *ahrs_thread(void *thread_id);


#endif // _UNAV_AHRS_H
