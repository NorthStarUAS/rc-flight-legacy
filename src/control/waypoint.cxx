// waypoint.cxx -- Class to hold data and return info relating to a waypoint
//
// Written by Curtis Olson, started September 2000.
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

#include <stdio.h>

#include <include/globaldefs.h>
#include <math/SGMath.hxx>

#include "waypoint.hxx"


// Constructor
SGWayPoint::SGWayPoint( const double field1, const double field2,
                        const double alt_m, const double agl_m,
                        const double speed_kt,
			const double bank_deg,
                        const modetype m, const string& s ):
    mode( ABSOLUTE ),
    target_lon( 0.0 ),
    target_lat( 0.0 ),
    target_alt_m( -9999.9 ),
    target_agl_m( -9999.9 ),
    target_speed_kt( 0.0 ),
    target_bank_deg( 0.0 ),
    offset_hdg_deg( 0.0 ),
    offset_dist_m( 0.0 ),
    distance( 0.0 ),
    id( "" )
 {
    mode = m;
    if ( mode == ABSOLUTE ) {
	target_lon = field1;
	target_lat = field2;
    } else if ( mode == RELATIVE ) {
	offset_hdg_deg = field1;
	offset_dist_m = field2;
    }
    target_alt_m = alt_m;
    target_agl_m = agl_m;
    target_speed_kt = speed_kt;
    target_bank_deg = bank_deg;
    id = s;
}


SGWayPoint::SGWayPoint( pyPropertyNode *config ):
    mode( INVALID ),
    target_lon( 0.0 ),
    target_lat( 0.0 ),
    target_alt_m( -9999.9 ),
    target_agl_m( -9999.9 ),
    target_speed_kt( 0.0 ),
    target_bank_deg( 0.0 ),
    offset_hdg_deg( 0.0 ),
    offset_dist_m( 0.0 ),
    distance( 0.0 ),
    id( "" )
{
    if ( config->hasChild("id") ) {
	id = config->getString("id");
    }
    if ( config->hasChild("lon") ) {
	target_lon = config->getDouble("lon");
	mode = ABSOLUTE;
    }
    if ( config->hasChild("lat") ) {
	target_lat = config->getDouble("lat");
	mode = ABSOLUTE;
    }
    if ( config->hasChild("alt_ft") ) {
	target_alt_m = config->getDouble("alt_ft") * SG_FEET_TO_METER;
    }
    if ( config->hasChild("agl_ft") ) {
	target_agl_m = config->getDouble("agl_ft") * SG_FEET_TO_METER;
    }
    if ( config->hasChild("speed_kt") ) {
	target_speed_kt = config->getDouble("speed_kt");
    }
    if ( config->hasChild("bank_deg") ) {
	target_bank_deg = config->getDouble("bank_deg");
    }
    if ( config->hasChild("offset_heading_deg") ) {
	offset_hdg_deg = config->getDouble("offset_heading_deg");
	mode = RELATIVE;
    }
    if ( config->hasChild("offset_dist_m") ) {
	offset_dist_m = config->getDouble("offset_dist_m");
	mode = RELATIVE;
    }
    if ( mode == INVALID ) {
	printf("Error in waypoint config logic, " );
	if ( id.length() ) {
	    printf("Section = %s", id.c_str() );
        }
    } else if ( mode == ABSOLUTE ) {
	printf("WPT: %.6f %.6f %.0f (MSL) %.0f (AGL) %.0f (kts)\n",
	       target_lon, target_lat, target_alt_m, target_agl_m,
	       target_speed_kt);
    } else if ( mode == RELATIVE ) {
	printf("WPT: %4.0f deg %.0fm %.0f (MSL) %.0f (AGL) %.0f (kts)\n",
	       offset_hdg_deg, offset_dist_m, target_alt_m, target_agl_m,
	       target_speed_kt);
    }
}


SGWayPoint::SGWayPoint():
    mode( ABSOLUTE ),
    target_lon( 0.0 ),
    target_lat( 0.0 ),
    target_alt_m( -9999.9 ),
    target_agl_m( -9999.9 ),
    target_speed_kt( 0.0 ),
    target_bank_deg( 0.0 ),
    offset_hdg_deg( 0.0 ),
    offset_dist_m( 0.0 ),
    distance( 0.0 ),
    id( "" )
{
}


// Destructor
SGWayPoint::~SGWayPoint() {
}


// Calculate course and distances.  For WGS84 and SPHERICAL
// coordinates lat, lon, and course are in degrees, alt and distance
// are in meters.
void SGWayPoint::CourseAndDistance( const double cur_lon,
				    const double cur_lat,
				    const double cur_alt,
				    double *course, double *dist ) const {
    SGGeoc current, target;
    current.setLongitudeDeg( cur_lon );
    current.setLatitudeDeg( cur_lat );
    target.setLongitudeDeg( target_lon );
    target.setLatitudeDeg( target_lat );
    *dist = SGGeodesy::distanceM( current, target );
    *course = 360.0 - SGGeodesy::courseRad( current, target ) * SG_RADIANS_TO_DEGREES;
}

// Calculate course and distances between two waypoints
void SGWayPoint::CourseAndDistance( const SGWayPoint &wp,
			double *course, double *dist ) const {
    CourseAndDistance( wp.get_target_lon(),
		       wp.get_target_lat(),
		       0.0 /* wp.get_target_alt_m() */ ,
		       course, dist );
}

/**
 * Update the target_lon and target_lat values of this waypoint
 * based on this waypoint's offset heading and offset distance
 * values.  The new target location is computed relative to the
 * provided reference point and reference heading.
 */
void SGWayPoint::update_relative_pos( const SGWayPoint &ref,
                                      const double ref_heading_deg )
{
    SGGeoc orig;
    orig.setLongitudeDeg( ref.get_target_lon() );
    orig.setLatitudeDeg( ref.get_target_lat() );

    double course = ref_heading_deg + offset_hdg_deg;
    if ( course < 0.0 ) { course += 360.0; }
    if ( course > 360.0 ) { course -= 360.0; }
    course = 360.0 - course; // invert to make this routine happy

    SGGeoc result;
    SGGeodesy::advanceRadM( orig, course * SGD_DEGREES_TO_RADIANS,
			    offset_dist_m, result );

    target_lon = result.getLongitudeDeg();
    target_lat = result.getLatitudeDeg();

    printf("ref_hdg = %.1f offset=%.1f course=%.1f dist=%.1f\n",
	   ref_heading_deg, offset_hdg_deg,
	   360.0 - course, offset_dist_m);
    printf("ref = %.6f %.6f  new = %.6f %.6f\n",
	   ref.get_target_lon(), ref.get_target_lat(),
	   target_lon, target_lat);
}
