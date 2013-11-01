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

#include "comms/display.h"
#include "comms/logging.h"
#include "comms/remote_link.h"
#include "include/globaldefs.h"
#include "main/globals.hxx"
#include "mission/mission_mgr.hxx"
#include "props/props_io.hxx"
#include "sensors/gps_mgr.hxx"
#include "util/exception.hxx"
#include "util/sg_path.hxx"
#include "util/wind.hxx"

#include "waypoint.hxx"
#include "route_mgr.hxx"


FGRouteMgr::FGRouteMgr() :
    active( new SGRoute ),
    standby( new SGRoute ),
    config_props( NULL ),
    bank_limit_node( NULL ),
    heading_gain_node( NULL ),
    L1_gain_node( NULL ),
    xtrack_gain_node( NULL ),
    lon_node( NULL ),
    lat_node( NULL ),
    alt_node( NULL ),
    groundspeed_node( NULL ),
    groundtrack_node( NULL ),
    target_heading_error_deg( NULL ),
    target_course_deg( NULL ),
    ap_roll_node( NULL ),
    target_agl_node( NULL ),
    override_agl_node( NULL ),
    target_msl_node( NULL ),
    override_msl_node( NULL ),
    target_waypoint( NULL ),
    wp_dist_m( NULL ),
    wp_eta_sec( NULL ),
    xtrack_dist_m( NULL ),
    proj_dist_m( NULL ),
    start_mode( FIRST_WPT ),
    follow_mode( XTRACK_LEG_HDG ),
    completion_mode( LOOP ),
    dist_remaining_m( 0.0 )
{
}


FGRouteMgr::~FGRouteMgr() {
    delete standby;
    delete active;
}


// bind property nodes
void FGRouteMgr::bind() {
    bank_limit_node = fgGetNode("/mission/route/bank-limit-deg", true);
    heading_gain_node = fgGetNode("/mission/route/heading-error-gain", true);
    L1_gain_node = fgGetNode("/mission/route/L1-gain", true);
    xtrack_gain_node = fgGetNode( "/mission/route/xtrack-steer-gain", true );

    lon_node = fgGetNode( "/position/longitude-deg", true );
    lat_node = fgGetNode( "/position/latitude-deg", true );
    alt_node = fgGetNode( "/position/altitude-ft", true );
    groundspeed_node = fgGetNode("/velocity/groundspeed-ms", true);
    groundtrack_node = fgGetNode( "/orientation/groundtrack-deg", true );

    ap_roll_node = fgGetNode("/autopilot/settings/target-roll-deg", true);
    target_course_deg = fgGetNode( "/autopilot/settings/target-groundtrack-deg", true );
    target_msl_node = fgGetNode( "/autopilot/settings/target-msl-ft", true );
    override_msl_node
	= fgGetNode( "/autopilot/settings/override-msl-ft", true );
    target_agl_node = fgGetNode( "/autopilot/settings/target-agl-ft", true );
    override_agl_node
	= fgGetNode( "/autopilot/settings/override-agl-ft", true );
    target_waypoint
	= fgGetNode( "/mission/route/target-waypoint-idx", true );
    wp_dist_m = fgGetNode( "/mission/route/wp-dist-m", true );
    wp_eta_sec = fgGetNode( "/mission/route/wp-eta-sec", true );
    xtrack_dist_m = fgGetNode( "/mission/route/xtrack-dist-m", true );
    proj_dist_m = fgGetNode( "/mission/route/projected-dist-m", true );
}


void FGRouteMgr::init( SGPropertyNode *branch ) {
    config_props = branch;

    bind();

    active->clear();
    standby->clear();

    if ( config_props != NULL ) {
	if ( ! build() ) {
	    printf("Detected an internal inconsistency in the route\n");
	    printf(" configuration.  See earlier errors for\n" );
	    printf(" details.");
	    exit(-1);
	}

	// build() constructs the new route in the "standby" slot,
	// swap it to "active"
	swap();
    }
}


void FGRouteMgr::update() {
    double direct_course, direct_distance;
    double leg_course, leg_distance;

    double nav_course = 0.0;
    double nav_dist_m = 0.0;

    double target_agl_m = 0.0;
    double target_msl_m = 0.0;

    if ( active->size() > 0 ) {
	if ( GPS_age() < 10.0 ) {

	    // route start up logic: if start_mode == FIRST_WPT then
	    // there is nothing to do, we simply continue to track wpt
	    // 0 if that is the current waypoint.  If start_mode ==
	    // FIRST_LEG, then if we are tracking wpt 0, then
	    // increment it so we track the 2nd waypoint along the
	    // first leg.  If you have provided a 1 point route and
	    // request first_leg startup behavior, then don't do that
	    // again, force sane route parameters instead!
	    if ( (start_mode == FIRST_LEG)
		 && (active->get_waypoint_index() == 0) ) {
		if ( active->size() > 1 ) {
		    active->increment_current();
		} else {
		    start_mode = FIRST_WPT;
		    follow_mode = DIRECT;
		}
	    }

	    // track current waypoint of route (only if we have fresh gps data)
	    SGWayPoint prev = active->get_previous();
	    SGWayPoint wp = active->get_current();

	    // compute direct-to course and distance
	    wp.CourseAndDistance( lon_node->getDoubleValue(),
				  lat_node->getDoubleValue(),
				  alt_node->getDoubleValue(),
				  &direct_course, &direct_distance );

	    // compute leg course and distance
	    wp.CourseAndDistance( prev, &leg_course, &leg_distance );

	    // difference between ideal (leg) course and direct course
            double angle = leg_course - direct_course;
            if ( angle < -180.0 ) {
	        angle += 360.0;
            } else if ( angle > 180.0 ) {
	        angle -= 360.0;
            }

	    // compute cross-track error
	    double angle_rad = angle * SGD_DEGREES_TO_RADIANS;
            double xtrack_m = sin( angle_rad ) * direct_distance;
            double dist_m = cos( angle_rad ) * direct_distance;
	    /* printf("direct_dist = %.1f angle = %.1f dist_m = %.1f\n",
	              direct_distance, angle, dist_m); */
	    xtrack_dist_m->setDoubleValue( xtrack_m );
	    proj_dist_m->setDoubleValue( dist_m );

	    // compute cross-track steering compensation
	    double xtrack_gain = xtrack_gain_node->getDoubleValue();
	    double xtrack_comp = xtrack_m * xtrack_gain;
	    if ( xtrack_comp < -45.0 ) { xtrack_comp = -45.0; }
	    if ( xtrack_comp > 45.0 ) { xtrack_comp = 45.0; }

	    // default distance for waypoint acquisition, direct
	    // distance to the target waypoint.  This can be overrided
	    // later by leg following and replaced with distance
	    // remaining along the leg.
	    nav_dist_m = direct_distance;

	    if ( follow_mode == DIRECT ) {
		// direct to
		nav_course = direct_course;
	    } else {
		// cross track steering
		if ( (active->get_waypoint_index() == 0)
		     && (completion_mode != LOOP) ) {
		    // first waypoint is 'direct to' except for LOOP
		    // routes which track the leg connecting the last
		    // wpt to the first wpt.
		    nav_course = direct_course;
		} else if ( active->get_waypoint_index() == active->size()-1 ) {
		    // force leg heading logic on last leg so it is
		    // possible to extend the center line beyond the
		    // waypoint.
		    nav_dist_m = dist_m;
		    nav_course = leg_course - xtrack_comp;
		} else if ( fabs(angle) <= 45.0 ) {
		    // normal case
		    nav_dist_m = dist_m;
		    if ( follow_mode == XTRACK_LEG_HDG ) {
			// xtrack_course based on leg_course should be
			// the most stable
			nav_course = leg_course - xtrack_comp;
		    } else if ( follow_mode == XTRACK_DIRECT_HDG ) {
			// xtrack_course based on direct_course could
			// start oscillating the closer we get to the
			// target waypoint.
			nav_course = direct_course - xtrack_comp;
		    }
		} else {
		    // navigate 'direct to' if off by more than the
		    // xtrack system can compensate for
		    nav_course = direct_course;
		}
	    }

            if ( nav_course < 0.0 ) {
                nav_course += 360.0;
            } else if ( nav_course > 360.0 ) {
                nav_course -= 360.0;
            }

	    target_course_deg->setDoubleValue( nav_course );

	    // target bank angle computed here

	    double target_bank_deg = 0.0;

#if 0 // original linear response to error

	    // compute heading error in aircraft heading space after
	    // doing wind triangle math on the current and target
	    // ground courses.  This gives us a close estimate of how
	    // far we have to yaw the aircraft nose to get on the
	    // target ground course.
	    double hdg_error
		= wind_heading_diff( groundtrack_node->getDoubleValue(),
				     target_course_deg->getDoubleValue() );
	    target_bank_deg = hdg_error * heading_gain_node->getDoubleValue();

#else // new L1 'mathematical' response to error

	    double L1 = L1_gain_node->getDoubleValue();	// gain
	    double course_error = groundtrack_node->getDoubleValue()
		- target_course_deg->getDoubleValue();
	    if ( course_error < -180.0 ) { course_error += 360.0; }
	    if ( course_error >  180.0 ) { course_error -= 360.0; }
	    double accel = 2.0 * groundspeed_node->getDoubleValue()
		* sin(course_error * SG_DEGREES_TO_RADIANS) / L1;

	    static const double gravity = 9.81; // m/sec^2
	    double target_bank = -atan( accel / gravity );
	    target_bank_deg = target_bank * SG_RADIANS_TO_DEGREES;
#endif

	    double bank_limit_deg = bank_limit_node->getDoubleValue();
	    if ( target_bank_deg < -bank_limit_deg ) {
		target_bank_deg = -bank_limit_deg;
	    }
	    if ( target_bank_deg > bank_limit_deg ) {
		target_bank_deg = bank_limit_deg;
	    }
	    ap_roll_node->setDoubleValue( target_bank_deg );

	    target_agl_m = wp.get_target_agl_m();
	    target_msl_m = wp.get_target_alt_m();

	    // estimate distance remaining to completion of route
	    dist_remaining_m = nav_dist_m
		+ active->get_remaining_distance_from_current_waypoint();

#if 0
	    if ( display_on ) {
		printf("next leg: %.1f  to end: %.1f  wpt=%d of %d\n",
		       nav_dist_m, dist_remaining_m,
		       active->get_waypoint_index(), active->size());
	    }
#endif

	    // logic to mark completion of leg and move to next leg.
	    if ( completion_mode == LOOP ) {
		if ( nav_dist_m < 50.0 ) {
		    active->increment_current();
		}
	    } else if ( completion_mode == CIRCLE_LAST_WPT ) {
		if ( nav_dist_m < 50.0 ) {
		    if ( active->get_waypoint_index() < active->size() - 1 ) {
			active->increment_current();
		    } else {
			SGWayPoint wp = active->get_current();
			mission_mgr.request_task_circle(wp.get_target_lon(),
							wp.get_target_lat());
		    }
		}
	    } else if ( completion_mode == EXTEND_LAST_LEG ) {
		if ( nav_dist_m < 50.0 ) {
		    if ( active->get_waypoint_index() < active->size() - 1 ) {
			active->increment_current();
		    } else {
			// follow the last leg forever
		    }
		}
	    }

	    // publish current target waypoint
	    target_waypoint->setIntValue( active->get_waypoint_index() );

	    // if ( display_on ) {
	    // printf("route dist = %0f\n", dist_remaining_m);
	    // }
	}
    } else {
        // FIXME: we've been commanded to follow a route, but no route
        // has been defined.

        // We are in ill-defined territory, should we do some sort of
        // circle of our home position?

	mission_mgr.request_task_circle();
    }

    wp_dist_m->setFloatValue( direct_distance );

    // update target altitude based on waypoint targets and possible
    // overrides ... preference is given to agl if both agl & msl are
    // set.
    double override_agl_ft = override_agl_node->getDoubleValue();
    double override_msl_ft = override_msl_node->getDoubleValue();
    if ( override_agl_ft > 1.0 ) {
	target_agl_node->setDoubleValue( override_agl_ft );
    } else if ( override_msl_ft > 1.0 ) {
	target_msl_node->setDoubleValue( override_msl_ft );
    } else if ( target_agl_m > 1.0 ) {
	target_agl_node->setDoubleValue( target_agl_m * SG_METER_TO_FEET );
    } else if ( target_msl_m > 1.0 ) {
	target_msl_node->setDoubleValue( target_msl_m * SG_METER_TO_FEET );
    }

    double gs_mps = groundspeed_node->getDoubleValue();
    if ( gs_mps > 0.1 ) {
	wp_eta_sec->setFloatValue( direct_distance / gs_mps );
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
}


bool FGRouteMgr::swap() {
    if ( !standby->size() ) {
	// standby route is empty
	return false;
    }

    // swap standby <=> active routes
    SGRoute *tmp = active;
    active = standby;
    standby = tmp;

    // set target way point to the first waypoint in the new active route
    active->set_current( 0 );

    return true;
}


bool FGRouteMgr::build() {
    standby->clear();

    SGPropertyNode *node;
    int i;

    int count = config_props->nChildren();
    for ( i = 0; i < count; ++i ) {
        node = config_props->getChild(i);
        string name = node->getName();
        // cout << name << endl;
        if ( name == "wpt" ) {
            SGWayPoint wpt( node );
            standby->add_waypoint( wpt );
	} else if ( name == "enable" ) {
	    // happily ignore this
        } else {
            printf("Unknown top level section: %s\n", name.c_str() );
            return false;
        }
    }

    printf("loaded %d waypoints\n", standby->size());

    return true;
}


int FGRouteMgr::new_waypoint( const string& wpt_string )
{
    SGWayPoint wp = make_waypoint( wpt_string );
    standby->add_waypoint( wp );
    return 1;
}


int FGRouteMgr::new_waypoint( const double field1, const double field2,
			      const double alt,
			      const int mode )
{
    if ( mode == 0 ) {
        // relative waypoint
	SGWayPoint wp( field2, field1, -9999.0, alt, 0.0, 0.0,
		       SGWayPoint::RELATIVE, "" );
	standby->add_waypoint( wp );
    } else if ( mode == 1 ) {
	// absolute waypoint
	SGWayPoint wp( field1, field2, -9999.0, alt, 0.0, 0.0,
		       SGWayPoint::ABSOLUTE, "" );
	standby->add_waypoint( wp );
    }

    return 1;
}


SGWayPoint FGRouteMgr::make_waypoint( const string& wpt_string ) {
    string target = wpt_string;
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
    SGWayPoint wp( lon, lat, alt_m, agl_m, speed_kt, 0.0,
                   SGWayPoint::ABSOLUTE, "" );

    return wp;
}


bool FGRouteMgr::reposition_pattern( const SGWayPoint &wp, const double hdg )
{
    // sanity check
    if ( fabs(wp.get_target_lon() > 0.0001)
	 || fabs(wp.get_target_lat() > 0.0001) )
    {
	// good location
	active->refresh_offset_positions( wp, hdg );
	if ( display_on ) {
	    printf( "ROUTE pattern updated: %.6f %.6f (course = %.1f)\n",
		    wp.get_target_lon(), wp.get_target_lat(),
		    hdg );
	}
	return true;
    } else {
	// bogus location, ignore ...
	return false;
    }
}
