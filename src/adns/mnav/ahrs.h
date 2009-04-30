/*******************************************************************************
 * FILE: ahrs.h
 * DESCRIPTION: attitude heading reference system providing the attitude of
 *   	       the vehicle using an extended Kalman filter
 ******************************************************************************/

#ifndef _UGEAR_AHRS_H
#define _UGEAR_AHRS_H


#include "props/props.hxx"

struct imu {
   double time;
   double p,q,r;		/* angular velocities    */
   double ax,ay,az;		/* acceleration          */
   double hx,hy,hz;             /* magnetic field     	 */
   double Ps,Pt;                /* static/pitot pressure */
   // double Tx,Ty,Tz;          /* temperature           */
   double phi,the,psi;          /* attitudes             */
   uint64_t status;		/* error type		 */
};


extern double xs[7];

void mnav_ahrs_init( SGPropertyNode *config );
void mnav_ahrs_update( struct imu *imupacket );
void mnav_ahrs_close();


#endif // _UGEAR_AHRS_H
