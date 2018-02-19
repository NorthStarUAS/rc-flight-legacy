/*! \file nav_functions.c
 *	\brief Auxiliary functions for nav filter
 *
 *	\details
 *     Module:          Navfuncs.c
 *     Modified:        Brian Taylor (convert to eigen3)
 *						Adhika Lie (revamp all functions)
 * 						Gokhan Inalhan (remaining)
 *                      Demoz Gebre (first three functions)
 *                      Jung Soon Jang
 *
 *     Description:     navfunc.c contains the listing for all the
 *                      real-time inertial navigation software.
 *
 *		Note: all the functions here do not create memory without
 *			  clearing it.
 *	\ingroup nav_fcns
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: nav_functions.c 922 2012-10-17 19:14:09Z joh07594 $
 */

/*     Include Pertinent Header Files */

#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using namespace Eigen;

#include "nav_functions_float.hxx"


// This function calculates the rate of change of latitude, longitude,
// and altitude using WGS-84.
Vector3f llaratef(Vector3f V, Vector3d lla) {
    double lat = lla(0,0);
    double h = lla(2,0);
	
    double denom = fabs(1.0 - (ECC2 * sin(lat) * sin(lat)));
    double sqrt_denom = sqrt(denom);
    
    double Rew = EARTH_RADIUS / sqrt_denom;
    double Rns = EARTH_RADIUS*(1-ECC2) / (denom*sqrt_denom);
	
    Vector3f lla_dot;
    lla_dot(0,0) = V(0,0)/(Rns + h);
    lla_dot(1,0) = V(1,0)/((Rew + h)*cos(lat));
    lla_dot(2,0) = -V(2,0);
	
    return lla_dot;
}

// This function converts a vector in ecef to ned coordinate centered
// at pos_ref.
Vector3f ecef2nedf(Vector3d ecef, Vector3d pos_ref) {
    double lat = pos_ref(0,0);
    double lon = pos_ref(1,0);
    double sin_lat = sin(lat);
    double sin_lon = sin(lon);
    double cos_lat = cos(lat);
    double cos_lon = cos(lon);
    
    Vector3f ned;
    ned(2,0) = -cos_lat*cos_lon*ecef(0,0) - cos_lat*sin_lon*ecef(1,0) - sin_lat*ecef(2,0);
    ned(1,0) = -sin_lon*ecef(0,0) + cos_lon*ecef(1,0);
    ned(0,0) = -sin_lat*cos_lon*ecef(0,0) - sin_lat*sin_lon*ecef(1,0) + cos_lat*ecef(2,0);
	
    return ned;
}

// This function gives a skew symmetric matrix from a given vector w
Matrix3f skf(Vector3f w) {
    Matrix3f C;

    C(0,0) = 0.0;	C(0,1) = -w(2,0);	C(0,2) = w(1,0);
    C(1,0) = w(2,0);	C(1,1) = 0.0;		C(1,2) = -w(0,0);
    C(2,0) = -w(1,0);	C(2,1) = w(0,0);	C(2,2) = 0.0;
	
    return C;
}

// Quaternion to euler angle: returns phi, the, psi as a vector
Vector3f quat2eulf(Quaternionf q) {
    float q0, q1, q2, q3;
    float m11, m12, m13, m23, m33;
	
    q0 = q.w();
    q1 = q.x();
    q2 = q.y();
    q3 = q.z();

    m11 = 2*(q0*q0 + q1*q1) - 1;
    m12 = 2*(q1*q2 + q0*q3);
    m13 = 2*(q1*q3 - q0*q2);
    m23 = 2*(q2*q3 + q0*q1);
    m33 = 2*(q0*q0 + q3*q3) - 1;
    
    Vector3f result;
    result(2) = atan2(m12,m11);
    result(1) = asin(-m13);
    result(0) = atan2(m23,m33);

    return result;
}

// Computes a quaternion from the given euler angles
Quaternionf eul2quatf(float phi, float the, float psi) {
    float sin_psi = sin(psi*0.5);
    float cos_psi = cos(psi*0.5);
    float sin_the = sin(the*0.5);
    float cos_the = cos(the*0.5);
    float sin_phi = sin(phi*0.5);
    float cos_phi = cos(phi*0.5);

    Quaternionf q;
    q.w() = cos_psi*cos_the*cos_phi + sin_psi*sin_the*sin_phi;  
    q.x() = cos_psi*cos_the*sin_phi - sin_psi*sin_the*cos_phi;
    q.y() = cos_psi*sin_the*cos_phi + sin_psi*cos_the*sin_phi;  
    q.z() = sin_psi*cos_the*cos_phi - cos_psi*sin_the*sin_phi;

    return q;
}

// Quaternion to C_N2B
Matrix3f quat2dcmf(Quaternionf q) {
    float q0, q1, q2, q3;
    Matrix3f C_N2B;

    q0 = q.w(); q1 = q.x(); q2 = q.y(); q3 = q.z();

    C_N2B(0,0) = 2*(q0*q0 + q1*q1) - 1;
    C_N2B(1,1) = 2*(q0*q0 + q2*q2) - 1;
    C_N2B(2,2) = 2*(q0*q0 + q3*q3) - 1;
	
    C_N2B(0,1) = 2*(q1*q2 + q0*q3);
    C_N2B(0,2) = 2*(q1*q3 - q0*q2);
	
    C_N2B(1,0) = 2*(q1*q2 - q0*q3);
    C_N2B(1,2) = 2*(q2*q3 + q0*q1);
	
    C_N2B(2,0) = 2*(q1*q3 + q0*q2);
    C_N2B(2,1) = 2*(q2*q3 - q0*q1);
	
    return C_N2B;
}
