// route.cxx -- Class to manage a list of waypoints (route)
//
// Written by Curtis Olson, started October 2000.
//
// Copyright (C) 2000  Curtis L. Olson  - curt@hfrl.umn.edu
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


#include "python/pyprops.hxx"

#include <math.h>

#include "route.hxx"


// constructor
SGRoute::SGRoute():
    current_wp(0),
    acquired(false)
{
    route.clear();
}


// destructor
SGRoute::~SGRoute() {
}


#if 0
// Calculate perpendicular distance from the current route segment
// This routine assumes all points are laying on a flat plane and
// ignores the altitude (or Z) dimension.  For best results, use with
// CARTESIAN way points.
double SGRoute::distance_off_route( double x, double y ) const {
    if ( current_wp > 0 ) {
	int n0 = current_wp - 1;
	int n1 = current_wp;
	sgdVec3 p, p0, p1, d;
	sgdSetVec3( p, x, y, 0.0 );
	sgdSetVec3( p0,
		    route[n0].get_target_lon(), route[n0].get_target_lat(),
		    0.0 );
	sgdSetVec3( p1,
		    route[n1].get_target_lon(), route[n1].get_target_lat(),
		    0.0 );
	sgdSubVec3( d, p0, p1 );

	return sqrt( sgdClosestPointToLineDistSquared( p, p0, d ) );

    } else {
	// We are tracking the first waypoint so there is no route
	// segment.  If you add the current location as the first
	// waypoint and the actual waypoint as the second, then we
	// will have a route segment and calculate distance from it.

	return 0;
    }
}
#endif


/** Update the length of the leg ending at waypoint index */
void SGRoute::update_distance(unsigned int index)
{
    SGWayPoint& curr = route[ index ];
    double course, dist;

    if ( index == 0 ) {
	dist = 0;
    } else {
	const SGWayPoint& prev = route[ index - 1 ];
	curr.CourseAndDistance( prev, &course, &dist );
    }

    curr.set_distance( dist );
}

/**
 * Add waypoint
 * @param wp a waypoint
 */
void SGRoute::add_waypoint( const SGWayPoint &wp ) {
    unsigned int n = route.size();
    route.push_back( wp );
    update_distance( n );
}

/**
 * Replace waypoint number 'n' with new waypoint.
 * @param wp a waypoint
 * @param n waypoint index number
 */
void SGRoute::replace_waypoint( const SGWayPoint &wp, unsigned int n ) {
    if ( n >= 0 && n < route.size() ) {
        route[n] = wp;
    }
    update_distance( n );
}

/** Delete waypoint with index n  (last one if n < 0) */
void SGRoute::delete_waypoint( unsigned int n ) {
    unsigned int size = route.size();
    if ( size == 0 ) {
        return;
    }

    if ( n < 0 || n >= size ) {
        n = size - 1;
    }

    route.erase( route.begin() + n );

    // update distance of next leg if not at end of route
    if ( n < size - 1 ) {
        update_distance( n );
    }
}


/** Refresh relative/offset positions of all waypoints in this
 *  route that have relative offsets.
 */
void SGRoute::refresh_offset_positions( const SGWayPoint &ref,
                                        const double ref_heading_deg )
{
    for ( unsigned int i = 0; i < route.size(); ++i ) {
	// positive value indicates 'relative' way point.  This
	// generally works ok, but it's conceivable we'd want a
	// waypoint at zero distance offset (exactly at home) so this
	// should be carefully rethought and a better way derived I
	// think.
        if ( route[i].get_mode() == SGWayPoint::RELATIVE ) {
            route[i].update_relative_pos( ref, ref_heading_deg );
	    update_distance( i );
        }
    }
}


/** Get the distance remaining in the route starting at the next
    (target) waypoint.  It is up to the calling layer to compute
    the distance to the next waypoint and add it to the remaining
    distance of the route.  The logic here is that the calling
    layer is already computing distance to the next waypoint, and
    then we need to know the distance from the next waypoint to
    the end to compute the total distance to the end of the route.
*/
double SGRoute::get_remaining_distance_from_current_waypoint() {
    double dist = 0.0;
    for ( unsigned int i = current_wp+1; i < route.size(); ++i ) {
	dist += route[i].get_distance();
    }
    return dist;
}
