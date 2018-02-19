/*! \file nav_functions.h
 *	\brief Auxiliary functions for nav filter header file
 *
 *	\details
 *     Module:          navfunc.h
 *     Modified:        Brian Taylor (convert to eigen3)
 *						Gokhan Inalhan (remaining) 
 *                      Demoz Gebre (first three functions)
 *                      Adhika Lie
 *                      Jung Soon Jang
 *     Description:     navfunc.h contains all the variable, 
 *                      constants and function prototypes that are 
 *                      used with the inertial navigation software.
 *	\ingroup nav_fcns
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: nav_functions.h 922 2012-10-17 19:14:09Z joh07594 $
 */

//#include "matrix.h"
#ifndef NAV_FUNCTIONS_FLOAT_HXX
#define NAV_FUNCTIONS_FLOAT_HXX


#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
using namespace Eigen;

#include "../nav_common/constants.hxx"


// This function calculates the rate of change of latitude, longitude,
// and altitude using WGS-84.
Vector3f llaratef(Vector3f V, Vector3d lla);

// This function converts a vector in ecef to ned coordinate centered
// at pos_ref.
Vector3f ecef2nedf(Vector3d ecef, Vector3d pos_ref);

// This function gives a skew symmetric matrix from a given vector w
Matrix3f skf(Vector3f w);

// Quaternion to euler angle: returns phi, the, psi as a vector
Vector3f quat2eulf(Quaternionf q);

// Computes a quaternion from the given euler angles
Quaternionf eul2quatf(float phi, float the, float psi);

// Quaternion to C_N2B
Matrix3f quat2dcmf(Quaternionf q);


#endif	// NAV_FUNCTIONS_FLOAT_HXX
