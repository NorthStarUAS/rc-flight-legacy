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
#include "comms/remote_link.h"
#include "main/globals.hxx"
#include "sensors/gps_mgr.h"

#include "waypoint.hxx"
#include "route_mgr.hxx"


FGRouteMgr route_mgr;           // global route manager object


FGRouteMgr::FGRouteMgr() :
    route( new SGRoute ),
    home_course_deg( 0.0 ),
    config_props( NULL ),
    lon_node( NULL ),
    lat_node( NULL ),
    alt_node( NULL ),
    vn_node( NULL ),
    ve_node( NULL ),
    true_hdg_deg( NULL ),
    target_agl_ft( NULL ),
    override_agl_ft( NULL ),
    target_msl_ft( NULL ),
    override_msl_ft( NULL ),
    target_waypoint( NULL ),
    wp_dist_m( NULL ),
    wp_eta_sec( NULL ),
    route_mode_node( NULL ),
    home_lon_node( NULL ),
    home_lat_node( NULL ),
    est_wind_speed_kt( NULL ),
    est_wind_dir_deg( NULL ),
    est_wind_east_mps( NULL ),
    est_wind_north_mps( NULL ),
    est_wind_true_heading_deg( NULL ),
    est_wind_target_heading_deg( NULL ),
    ap_console_skip( NULL ),
    ap_logging_skip( NULL ),
    home_set( false ),
    mode( GoHome )
{
}


FGRouteMgr::~FGRouteMgr() {
    delete route;
}


// bind property nodes
void FGRouteMgr::bind() {
    config_props = fgGetNode( "/config/route", true );

    lon_node = fgGetNode( "/position/longitude-deg", true );
    lat_node = fgGetNode( "/position/latitude-deg", true );
    alt_node = fgGetNode( "/position/altitude-ft", true );
    vn_node = fgGetNode( "/velocity/vn-ms", true);
    ve_node = fgGetNode( "/velocity/ve-ms", true);

    true_hdg_deg = fgGetNode( "/autopilot/settings/target-groundtrack-deg", true );
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
    wp_dist_m = fgGetNode( "/autopilot/route-mgr/wp-dist-m", true );
    wp_eta_sec = fgGetNode( "/autopilot/route-mgr/wp-eta-m", true );

    route_mode_node = fgGetNode("/routes/mode", true);

    home_lon_node = fgGetNode("/routes/home/longitude-deg", true );
    home_lat_node = fgGetNode("/routes/home/latitude-deg", true );

    est_wind_speed_kt = fgGetNode("/filters/wind-est/wind-speed-kt", true);
    est_wind_dir_deg = fgGetNode("/filters/wind-est/wind-dir-deg", true);
    est_wind_east_mps = fgGetNode("/filters/wind-est/wind-east-mps", true);
    est_wind_north_mps = fgGetNode("/filters/wind-est/wind-north-mps", true);
    est_wind_true_heading_deg
	= fgGetNode("/filters/wind-est/true-heading-deg", true);
    est_wind_target_heading_deg
	= fgGetNode("/filters/wind-est/target-heading-deg", true);

    ap_console_skip = fgGetNode("/config/remote-link/autopilot-skip", true);
    ap_logging_skip = fgGetNode("/config/logging/autopilot-skip", true);
}


void FGRouteMgr::init() {
    bind();

    route->clear();

    if ( ! build() ) {
	printf("Detected an internal inconsistency in the route\n");
	printf(" configuration.  See earlier errors for\n" );
	printf(" details.");
	exit(-1);
    }

    set_route_mode();
}


void FGRouteMgr::update() {
    double wp_course, wp_distance;

    double override_agl = override_agl_ft->getDoubleValue();
    double override_msl = override_msl_ft->getDoubleValue();
    double target_agl_m = 0;
    double target_msl_m = 0;

    if ( mode == GoHome && home_set ) {
        home.CourseAndDistance( lon_node->getDoubleValue(),
				lat_node->getDoubleValue(),
                                alt_node->getDoubleValue(),
                                &wp_course, &wp_distance );

        true_hdg_deg->setDoubleValue( wp_course );
        target_agl_m = home.get_target_agl_m();
        target_msl_m = home.get_target_alt_m();

	// publish current target waypoint
	target_waypoint->setIntValue( 0 );
    } else if ( mode == FollowRoute && route->size() > 0 ) {
	if ( GPS_age() < 10.0 ) {
	    // track current waypoint of route (only if we have fresh gps data)
	    SGWayPoint wp = route->get_current();
	    wp.CourseAndDistance( lon_node->getDoubleValue(),
				  lat_node->getDoubleValue(),
				  alt_node->getDoubleValue(),
				  &wp_course, &wp_distance );

	    true_hdg_deg->setDoubleValue( wp_course );
	    target_agl_m = wp.get_target_agl_m();
	    target_msl_m = wp.get_target_alt_m();

	    if ( wp_distance < 50.0 ) {
		route->increment_current();
	    }

	    // publish current target waypoint
	    target_waypoint->setIntValue( route->get_waypoint_index() );
	}
    } else {
        // FIXME: we've been commanded to go home and no home position
        // has been set, or we've been commanded to follow a route,
        // but no route has been defined.

        // We are in ill-defined territory, I'd like to go into some
        // sort of slow circling mode and either hold altitude or
        // maybe do a slow speed decent to minimize our momentum.
    }

    wp_dist_m->setFloatValue( wp_distance );

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

    // Compute wind based current heading estimate and target heading
    // estimate.  This can be used to "schedule" the bank angle gain
    // correctly so that up wind flying will produce gentler turns and
    // down wind flying will produce more aggressive turns.
    double vn = vn_node->getDoubleValue();
    double ve = ve_node->getDoubleValue();
    double wn = est_wind_north_mps->getDoubleValue();
    double we = est_wind_east_mps->getDoubleValue();

    double true_e = ve + we;
    double true_n = vn + wn;

    double true_speed_mps = sqrt( true_e*true_e + true_n*true_n );
    double true_deg = 90 - atan2( true_n, true_e ) * SGD_RADIANS_TO_DEGREES;
    if ( true_deg < 0 ) { true_deg += 360.0; }

    est_wind_true_heading_deg->setDoubleValue( true_deg );

    // from williams.best.vwh.net/avform.htm (aviation formulas)
    double ws = est_wind_speed_kt->getDoubleValue() * SG_KT_TO_MPS;
    double tas = true_speed_mps;
    double wd = est_wind_dir_deg->getDoubleValue() * SGD_DEGREES_TO_RADIANS;
    double crs = wp_course * SGD_DEGREES_TO_RADIANS;
    double hd = 0.0;
    double gs = 0.0;
    if ( tas > 0.1 ) {
	// printf("ws=%.1f tas=%.1f wd=%.1f crs=%.1f\n", ws, tas, wd, crs);
	double swc = (ws/tas)*sin(wd-crs);
	// printf("swc=%.2f\n", swc);
	if ( fabs(swc) > 1.0 ) {
	    // course cannot be flown, wind too strong
	    // point nose into estimated wind and "kite" as best we can
	    hd = wd + SGD_PI;
	    if ( hd > SGD_2PI ) { hd -= SGD_2PI; }
	} else {
	    hd = crs + asin(swc);
	    if ( hd < 0.0 ) { hd += SGD_2PI; }
	    if ( hd > SGD_2PI ) { hd -= SGD_2PI; }
	    gs = tas * sqrt(1-swc*swc) - ws * cos(wd - crs);
	}
	est_wind_target_heading_deg
	    ->setDoubleValue( hd * SGD_RADIANS_TO_DEGREES );
    }
    // if ( display_on ) {
    //   printf("af: hd=%.1f gs=%.1f\n", hd * SGD_RADIANS_TO_DEGREES, gs);
    // }

    if ( gs > 0.1 ) {
	wp_eta_sec->setFloatValue( wp_distance / (gs * SG_KT_TO_MPS) );
    } else {
	wp_eta_sec->setFloatValue( 0.0 );
    }

#if 0
    if ( display_on ) {
	SGPropertyNode *ground_deg = fgGetNode("/orientation/groundtrack-deg", true);
	double gtd = ground_deg->getDoubleValue();
	if ( gtd < 0 ) { gtd += 360.0; }
	double diff = wp_course - gtd;
	if ( diff < -180.0 ) { diff += 360.0; }
	if ( diff > 180.0 ) { diff -= 360.0; }
	SGPropertyNode *psi = fgGetNode("/orientation/heading-deg", true);
	printf("true filt=%.1f true-wind-est=%.1f target-hd=%.1f\n",
	       psi->getDoubleValue(), true_deg, hd * SGD_RADIANS_TO_DEGREES);
	printf("gt cur=%.1f target=%.1f diff=%.1f\n", gtd, wp_course, diff);
	diff = hd*SGD_RADIANS_TO_DEGREES - true_deg;
	if ( diff < -180.0 ) { diff += 360.0; }
	if ( diff > 180.0 ) { diff -= 360.0; }
	printf("wnd: cur=%.1f target=%.1f diff=%.1f\n",
	       true_deg, hd * SGD_RADIANS_TO_DEGREES, diff);
    }
#endif

    if ( remote_link_on || log_to_file ) {
	// send one waypoint per message, then home location (with
	// index = 65535)
	static int wp_index = 0;
	int index = 0;
	SGWayPoint wp;
	if ( size() > 0 && wp_index < size() ) {
	    wp = route_mgr.get_waypoint( wp_index );
	    index = wp_index;
	} else {
	    wp = route_mgr.get_home();
	    index = 65535;
	}

	uint8_t buf[256];
	int pkt_size = packetizer->packetize_ap( buf, &wp, index );
	
	if ( remote_link_on ) {
	    bool result = remote_link_ap( buf, pkt_size,
					   ap_console_skip->getIntValue() );
	    if ( result ) {
		wp_index++;
		if ( wp_index >= size() + 1 ) {
		    wp_index = 0;
		}
	    }
	}

	if ( log_to_file ) {
	    log_ap( buf, pkt_size, ap_logging_skip->getIntValue() );
	}
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
	} else if ( name == "enable" ) {
	    // happily ignore this
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
	    home_lon_node->setDoubleValue( home.get_target_lon() );
	    home_lat_node->setDoubleValue( home.get_target_lat() );
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


SGWayPoint FGRouteMgr::get_home() {
    if ( home_set ) {
	return home;
    } else {
	return SGWayPoint();
    }
}


void FGRouteMgr::set_route_mode() {
    mode = FollowRoute;
    route_mode_node->setStringValue("route");
    /*
      FILE *debug = fopen("/mnt/mmc/debug.txt", "a");
      fprintf(debug, "mode: FollowRoute\n");
      fclose(debug);
    */
}


void FGRouteMgr::set_home_mode() {
    mode = GoHome;
    route_mode_node->setStringValue("home");
    /*
      FILE *debug = fopen("/mnt/mmc/debug.txt", "a");
      fprintf(debug, "mode: GoHome\n");
      fclose(debug);
    */
}

