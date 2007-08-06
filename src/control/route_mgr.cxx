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
// $Id: route_mgr.cxx,v 1.1 2007/08/06 19:41:48 curt Exp $


#include <include/globaldefs.h>
#include <props/props_io.hxx>
#include <util/exception.hxx>
#include <util/sg_path.hxx>

#include "waypoint.hxx"
#include "route_mgr.hxx"


FGRouteMgr::FGRouteMgr() :
    route( new SGRoute ),
    config_props( NULL ),
    lon( NULL ),
    lat( NULL ),
    alt( NULL ),
    true_hdg_deg( NULL ),
    target_altitude_ft( NULL ),
    altitude_set( false )
{
}


FGRouteMgr::~FGRouteMgr() {
    delete route;
}


void FGRouteMgr::init() {
    config_props = fgGetNode( "/route[0]", true );

    SGPropertyNode *root_n = fgGetNode("/config/root-path");
    SGPropertyNode *path_n = fgGetNode("/config/route/path");

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
    target_altitude_ft = fgGetNode( "/autopilot/settings/target-altitude-ft", true );

    route->clear();
}


void FGRouteMgr::update( double dt ) {
    double wp_course, wp_distance;

    // first way point
    if ( route->size() > 0 ) {
        SGWayPoint wp = route->get_waypoint( 0 );
        wp.CourseAndDistance( lon->getDoubleValue(), lat->getDoubleValue(),
                              alt->getDoubleValue(), &wp_course, &wp_distance );

        true_hdg_deg->setDoubleValue( wp_course );
        double target_alt_m = wp.get_target_alt_m();

        if ( !altitude_set && target_alt_m > -9990 ) {
            target_altitude_ft->setDoubleValue( target_alt_m * SG_METER_TO_FEET );
            altitude_set = true;
        }

        if ( wp_distance < 200.0 ) {
            pop_waypoint();
            altitude_set = false;
        }
    }

}


void FGRouteMgr::add_waypoint( const SGWayPoint& wp, int n ) {
    if ( n == 0 || !route->size() )
        altitude_set = false;

    route->add_waypoint( wp, n );
}


SGWayPoint FGRouteMgr::pop_waypoint( int n ) {
    SGWayPoint wp;

    if ( route->size() > 0 ) {
        if ( n < 0 )
            n = route->size() - 1;
        wp = route->get_waypoint(n);
        route->delete_waypoint(n);
    }

    if ( n == 0 && route->size() )
        altitude_set = false;

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

    return true;
}


int FGRouteMgr::new_waypoint( const string& target, int n ) {
    SGWayPoint wp = make_waypoint( target );
    add_waypoint( wp, n );
    return 1;
}


SGWayPoint FGRouteMgr::make_waypoint( const string& tgt ) {
    string target = tgt;
    double lon = 0.0;
    double lat = 0.0;
    double alt_m = -9999.0;
    double speed_kt = 0.0;

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

        return 1;
    }

    printf("Adding waypoint lon = %.6f lat = %.6f\n", lon, lat);
    SGWayPoint wp( lon, lat, alt_m, speed_kt, SGWayPoint::SPHERICAL, "" );

    return wp;
}
