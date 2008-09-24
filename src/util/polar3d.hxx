/**
 * \file polar3d.hxx
 * Routines to deal with polar math and transformations.
 */

// Written by Curtis Olson, started June 1997.
//
// Copyright (C) 1997  Curtis L. Olson  - http://www.flightgear.org/~curt
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Library General Public
// License as published by the Free Software Foundation; either
// version 2 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Library General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//
// $Id: polar3d.hxx,v 1.2 2008/09/24 21:30:49 curt Exp $


#ifndef _POLAR3D_HXX
#define _POLAR3D_HXX


#ifndef __cplusplus
# error This library requires C++
#endif


#include "point3d.hxx"

/**
 * Calculate new lon/lat given starting lon/lat, and offset radial, and
 * distance.  NOTE: starting point is specifed in radians, distance is
 * specified in meters (and converted internally to radians)
 * ... assumes a spherical world.
 * @param orig specified in polar coordinates
 * @param course offset radial (in radians)
 * @param dist offset distance
 * @return destination point in polar coordinates
 */
Point3D calc_gc_lon_lat( const Point3D& orig, double course, double dist );


/**
 * Calculate course/dist given two spherical points.
 * @param start starting point
 * @param dest ending point
 * @param course resulting course
 * @param dist resulting distance
 */
void calc_gc_course_dist( const Point3D& start, const Point3D& dest, 
                                 double *course, double *dist );

#if 0
/**
 * Calculate course/dist given two spherical points.
 * @param start starting point
 * @param dest ending point
 * @param course resulting course
 * @param dist resulting distance
 */
void calc_gc_course_dist( const Point3D& start, const Point3D& dest, 
				 double *course, double *dist );
#endif // 0

#endif // _POLAR3D_HXX

