// route_mgr.cxx - manage a route (i.e. a collection of waypoints)
//
// Written by Curtis Olson, started January 2004.
//            Norman Vine
//            Melchior FRANZ
//
// Copyright (C) 2004  Curtis L. Olson  - http://www.flightgear.org/~curt
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
// $Id: route_mgr.cxx,v 1.14 2009/04/13 15:29:48 curt Exp $


#include <math.h>
#include <stdlib.h>

#include <include/globaldefs.h>
#include <props/props_io.hxx>
#include <util/exception.hxx>
#include <util/sg_path.hxx>

#include "comms/logging.h"

#include "waypoint.hxx"
#include "route_mgr.hxx"


FGRouteMgr route_mgr;           // global route manager object


FGRouteMgr::FGRouteMgr() :
    route( new SGRoute ),
    home_course_deg( 0.0 ),
    config_props( NULL ),
    lon( NULL ),
    lat( NULL ),
    alt( NULL ),
    true_hdg_deg( NULL ),
    target_agl_ft( NULL ),
    override_agl_ft( NULL ),
    target_msl_ft( NULL ),
    override_msl_ft( NULL ),
    target_waypoint( NULL ),
    home_set( false ),
    mode( GoHome )
{
}


FGRouteMgr::~FGRouteMgr() {
    delete route;
}


void FGRouteMgr::init() {
    config_props = fgGetNode( "/route[0]", true );

    SGPropertyNode *root_n = fgGetNode("/config/root-path");
    SGPropertyNode *path_n = fgGetNode("/config/route/path");

    route->clear();

    if ( path_n ) {
        SGPath config( root_n->getStringValue() );
        config.append( path_n->getStringValue() );

        printf("Reading route configuration from %s\n", config.c_str() );
        try {
            readProperties( config.str(), config_props );

            if ( ! build() ) {
	        printf("Detected an internal inconsistency in the route\n");
                printf(" configuration.  See earlier errors for\n" );
                printf(" details.");
                exit(-1);
            }

            mode = FollowRoute;
        } catch (const sg_exception& exc) {
            printf("Failed to load route configuration: %s\n",
                   config.c_str());
        }

    } else {
        printf("No autopilot configuration specified in master.xml file!");
    }

    lon = fgGetNode( "/position/longitude-deg", true );
    lat = fgGetNode( "/position/latitude-deg", true );
    alt = fgGetNode( "/position/altitude-ft", true );

    true_hdg_deg = fgGetNode( "/autopilot/settings/true-heading-deg", true );
    target_msl_ft
        = fgGetNode( "/autopilot/settings/target-msl-ft", true );
    override_msl_ft
        = fgGetNode( "/autopilot/settings/override-msl-ft", true );
    target_agl_ft
        = fgGetNode( "/autopilot/settings/target-agl-ft", true );
    override_agl_ft
        = fgGetNode( "/autopilot/settings/override-agl-ft", true );
    target_waypoint
	= fgGetNode( "/autopilot/route-mgr/target-waypoint-idx", true );
}


void FGRouteMgr::update() {
    double wp_course, wp_distance;

    double override_agl = override_agl_ft->getDoubleValue();
    double override_msl = override_msl_ft->getDoubleValue();
    double target_agl_m = 0;
    double target_msl_m = 0;

    if ( mode == GoHome && home_set ) {
        home.CourseAndDistance( lon->getDoubleValue(), lat->getDoubleValue(),
                                alt->getDoubleValue(),
                                &wp_course, &wp_distance );

        true_hdg_deg->setDoubleValue( wp_course );
        target_agl_m = home.get_target_agl_m();
        target_msl_m = home.get_target_alt_m();

	// publish current target waypoint
	target_waypoint->setIntValue( 0 );
    } else if ( mode == FollowRoute && route->size() > 0 ) {
        // track current waypoint of route
        SGWayPoint wp = route->get_current();
        wp.CourseAndDistance( lon->getDoubleValue(), lat->getDoubleValue(),
                              alt->getDoubleValue(), &wp_course, &wp_distance );

        true_hdg_deg->setDoubleValue( wp_course );
        target_agl_m = wp.get_target_agl_m();
        target_msl_m = wp.get_target_alt_m();

        if ( wp_distance < 50.0 ) {
            route->increment_current();
        }

        // publish current target waypoint
        target_waypoint->setIntValue( route->get_waypoint_index() );
    } else {
        // FIXME: we've been commanded to go home and no home position
        // has been set, or we've been commanded to follow a route,
        // but no route has been defined.

        // We are in ill-defined territory, I'd like to go into some
        // sort of slow circling mode and either hold altitude or
        // maybe do a slow speed decent to minimize our momentum.
    }

    // update target altitude based on waypoint targets and possible
    // overrides ... preference is given to agl if both agl & msl are
    // set.
    if ( override_agl > 1 ) {
	target_agl_ft->setDoubleValue( override_agl );
    } else if ( override_msl > 1 ) {
	target_msl_ft->setDoubleValue( override_msl );
    } else if ( target_agl_m > 1 ) {
	target_agl_ft->setDoubleValue( target_agl_m * SG_METER_TO_FEET );
    } else if ( target_msl_m > 1 ) {
	target_msl_ft->setDoubleValue( target_msl_m * SG_METER_TO_FEET );
    }

}


void FGRouteMgr::add_waypoint( const SGWayPoint& wp ) {
    route->add_waypoint( wp );
}


void FGRouteMgr::replace_waypoint( const SGWayPoint& wp, int n ) {
    if ( n >= 0 && n < route->size() ) {
        route->replace_waypoint( wp, n );
    }
}


SGWayPoint FGRouteMgr::pop_waypoint( int n ) {
    SGWayPoint wp;

    if ( route->size() > 0 ) {
        if ( n < 0 ) {
            n = route->size() - 1;
        }
        wp = route->get_waypoint(n);
        route->delete_waypoint(n);
    }

    return wp;
}


bool FGRouteMgr::build() {
    route->clear();

    SGPropertyNode *node;
    int i;

    int count = config_props->nChildren();
    for ( i = 0; i < count; ++i ) {
        node = config_props->getChild(i);
        string name = node->getName();
        // cout << name << endl;
        if ( name == "wpt" ) {
            SGWayPoint wpt( node );
            route->add_waypoint( wpt );
        } else {
            printf("Unknown top level section: %s\n", name.c_str() );
            return false;
        }
    }

    printf("loaded %d waypoints\n", route->size());

    return true;
}


int FGRouteMgr::new_waypoint( const string& target ) {
    SGWayPoint wp = make_waypoint( target );
    add_waypoint( wp );
    return 1;
}


SGWayPoint FGRouteMgr::make_waypoint( const string& tgt ) {
    string target = tgt;
    double lon = 0.0;
    double lat = 0.0;
    double alt_m = -9999.0;
    double agl_m = -9999.0;
    double speed_kt = 0.0;

    // WARNING: this routine doesn't have any way to handle AGL
    // altitudes.  Nor can it handle any offset heading/dist requests

    // extract altitude
    size_t pos = target.find( '@' );
    if ( pos != string::npos ) {
        alt_m = atof( target.c_str() + pos + 1 ) * SG_FEET_TO_METER;
        target = target.substr( 0, pos );
    }

    // check for lon,lat
    pos = target.find( ',' );
    if ( pos != string::npos ) {
        lon = atof( target.substr(0, pos).c_str());
        lat = atof( target.c_str() + pos + 1);
    }

    printf("Adding waypoint lon = %.6f lat = %.6f alt_m = %.0f\n",
           lon, lat, alt_m);
    SGWayPoint wp( lon, lat, alt_m, agl_m, speed_kt, 0.0, 0.0,
                   SGWayPoint::SPHERICAL, "" );

    return wp;
}


bool FGRouteMgr::update_home( const SGWayPoint &wp, const double hdg,
                              bool force_update )
{
    if ( !home_set || force_update ) {
        // sanity check
        if ( fabs(wp.get_target_lon() > 0.0001)
             || fabs(wp.get_target_lat() > 0.0001) )
        {
            // good location
            home = wp;
            home_course_deg = hdg;
            home_set = true;
            route->refresh_offset_positions( wp, home_course_deg );
            if ( display_on ) {
                printf( "HOME updated: %.6f %.6f (course = %.1f)\n",
                        home.get_target_lon(), home.get_target_lat(),
                        home_course_deg );
            }
            return true;
        } else {
            // bogus location, ignore ...
            return false;
        }
    }

    return false;
}
