/*-------------------------------------------------------------------
 *
 *     Module:          navfunc.h
 *     Modified:        Gokhan Inalhan (remaining) 
 *                      Demoz Gebre (first three functions)
 *                      Jung Soon Jang
 *     Description:     navfunc.h contains all the variable, 
 *                      constants and function prototypes that are 
 *                      used with the inertial navigation software.
 *
 *--------------------------------------------------------------------*/

#ifndef _UGEAR_NAVFUNC_H
#define _UGEAR_NAVFUNC_H

#include "matrix.h"

/*     Define Constants   */

#define EARTH_RATE   0.00007292115   /* rotation rate of earth (rad/sec) */
#define EARTH_RADIUS 6378137         /* earth semi-major axis radius (m) */
#define ECCENTRICITY 0.0818191908426 /* major eccentricity of earth ellipsoid */
#define FLATTENING   0.0033528106650 /* flattening of the ellipsoid */
#define GRAVITY_0    9.7803730       /* zeroth coefficient for gravity model */
#define GRAVITY_1    0.0052891       /* first coefficient for the gravity model*/ 
#define GRAVITY_2    0.0000059       /* second coefficient for the gravity model*/
#define GRAVITY_NOM  9.81            /* nominal gravity */ 
#define SCHULER2     1.533421593170545E-06 /* Sculer Frequency (rad/sec) Squared */
#define R2D          57.29577951308232     /* radians to degrees conversion factor */
#define D2R          0.01745329251994      /* degrees to radians conversion factor */  
#define FT2M         0.3048                /* feet to meters conversion factor */
#define KTS2ms       0.5144                /* Knots to meters/sec conversion factor*/
#define PI           3.14159265358979      /* pi */
// #define MAG_DEC     -0.270944862           /*magnetic declination of Stanford (rad): -15.15 */
#define MM2M         0.001                 /*mm to m*/

/*---------------     Define Structures and Enumerated Types -------------*/
typedef enum {OFF, ON} toggle;


/*
 * Function:     MATRIX EulerToDcm(MATRIX euler)
 *----------------------------------------------------------------------
 * Computer the direction cosine matrix that transforms a vector in
 * a reference axis system at time k to a reference axis system
 * at time k+1.  The input argument 'euler' is a vector containing the
 * the three euler angles in radians.  The order of the angles is assumed
 * to be yaw, pitch and roll (i.e., 3-2-1 rotation convention).
 */
MATRIX EulerToDcm(MATRIX euler, double dipA, MATRIX dcm);


/* Function    void EcefToEnu(MATRIX outputVector, MATRIX inputVector,
 *                              MATRIX position);
 *-------------------------------------------------------------
 * Converts the vector given in ECEF coordinates to a vector in 
 * ENU (East, North, Up) coordinates centered at the location
 * given in position (in lattitude, longitude, altitude);
 */ 
void EcefToEnu(MATRIX outputVector, MATRIX inputVector, MATRIX position);


/* Function    void EcefToLatLonAlt(MATRIX vector);
 *-----------------------------------------------------
 * Converts a position vector given in ECEF coordinates
 * into latitude, longitude and altitude.
 */
void EcefToLatLonAlt(MATRIX vector);

/* Function void LatLonAltToEcef(MATRIX vector, MATRIX position );
 *--------------------------------------------------------------
 * Converts a position vector given in lattitude, longitude and 
 * altitude to a vector in ECEF coordinates.
 */
void LatLonAltToEcef(MATRIX vector, MATRIX position);

/* Function void nCltrans(MATRIX n_C_l, double magdec)
 *--------------------------------------------------------------
 * Creates the transformation matrix from IMU level earth frame 
 * to instantenous Navigation ENU frame
 */
void nCltrans(MATRIX n_C_l, double magdec);

/* Function void eCntrans(MATRIX e_C_n, MATRIX LatLon)
 *--------------------------------------------------------------
 * Creates the transformation matrix from Navigation ENU frame
 * to ECEF frame through LatLon matrix=[Lat Lon]' (rad)
 */
void eCntrans(MATRIX e_C_n, MATRIX LatLon);

/* void lCbtrans(MATRIX l_C_b, MATRIX YawPitchRoll)
 *---------------------------------------------------------------
 * Creates the transformation matrix from B(b:body) to ELF(l:earth level) frame
 * input: yaw-psi(rad) pitch-theta(rad) roll-phi(rad) inverse of 3-2-1 Euler transformation
 */
void lCbtrans(MATRIX l_C_b, MATRIX YawPitchRoll); 
 
#endif // _UGEAR_NAVFUNC_H












