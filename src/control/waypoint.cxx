// waypoint.cxx -- Class to hold data and return info relating to a waypoint
//
// Written by Curtis Olson, started September 2000.
//
// Copyright (C) 2000  Curtis L. Olson  - curt@hfrl.umn.edu
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//
// $Id: waypoint.cxx,v 1.10 2009/08/25 15:04:01 curt Exp $


#include <stdio.h>

#include <include/globaldefs.h>
#include <math/SGMath.hxx>

// #include <util/polar3d.hxx>

#include "waypoint.hxx"


// Constructor
SGWayPoint::SGWayPoint( const double field1, const double field2,
                        const double alt_m, const double agl_m,
                        const double speed_kt,
                        const modetype m, const string& s ):
    mode( ABSOLUTE ),
    target_lon( 0.0 ),
    target_lat( 0.0 ),
    target_alt_m( -9999.9 ),
    target_agl_m( -9999.9 ),
    target_speed_kt( 0.0 ),
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
    id = s;
}


SGWayPoint::SGWayPoint( SGPropertyNode *node ):
    mode( ABSOLUTE ),
    target_lon( 0.0 ),
    target_lat( 0.0 ),
    target_alt_m( -9999.9 ),
    target_agl_m( -9999.9 ),
    target_speed_kt( 0.0 ),
    offset_hdg_deg( 0.0 ),
    offset_dist_m( 0.0 ),
    distance( 0.0 ),
    id( "" )
{
    int i;
    for ( i = 0; i < node->nChildren(); ++i ) {
        SGPropertyNode *child = node->getChild(i);
        string cname = child->getName();
        string cval = child->getStringValue();
        if ( cname == "id" ) {
            id = cval;
        } else if ( cname == "lon" ) {
            target_lon = child->getDoubleValue();
	    mode = ABSOLUTE;
        } else if ( cname == "lat" ) {
            target_lat = child->getDoubleValue();
	    mode = ABSOLUTE;
        } else if ( cname == "alt-ft" ) {
            target_alt_m = child->getDoubleValue() * SG_FEET_TO_METER;
        } else if ( cname == "agl-ft" ) {
            target_agl_m = child->getDoubleValue() * SG_FEET_TO_METER;
        } else if ( cname == "speed-kt" ) {
            target_speed_kt = child->getDoubleValue();
        } else if ( cname == "offset-heading-deg" ) {
            offset_hdg_deg = child->getDoubleValue();
	    mode = RELATIVE;
        } else if ( cname == "offset-dist-m" ) {
            offset_dist_m = child->getDoubleValue();
	    mode = RELATIVE;
        } else {
            printf("Error in waypoint config logic, " );
            if ( id.length() ) {
                printf("Section = %s", id.c_str() );
            }
        }
    }
    if ( mode == ABSOLUTE ) {
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

	 /*
	   FILE *debug = fopen("/mnt/mmc/debug.txt", "a");
	   fprintf(debug, "ref_hdg = %.1f offset=%.1f course=%.1f dist=%.1f\n",
                   ref_heading_deg, offset_hdg_deg,
                   360.0 - course, offset_dist_m);
           fprintf(debug, "ref = %.6f %.6f  new = %.6f %.6f\n",
                   ref.get_target_lon(), ref.get_target_lat(),
                   target_lon, target_lat);
           fclose(debug);
	 */
}
