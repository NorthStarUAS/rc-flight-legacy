// route_mgr.cxx - manage a route (i.e. a collection of waypoints)
//
// Written by Curtis Olson, started January 2004.
//            Norman Vine
//            Melchior FRANZ
//
// Copyright (C) 2004  Curtis L. Olson  - http://www.flightgear.org/~curt
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
#include <stdlib.h>

#include "comms/display.hxx"
#include "comms/logging.hxx"
#include "comms/remote_link.hxx"
#include "include/globaldefs.h"
#include "init/globals.hxx"
#include "sensors/gps_mgr.hxx"
#include "util/exception.hxx"
#include "util/sg_path.hxx"
#include "util/strutils.hxx"
#include "util/wind.hxx"

#include "waypoint.hxx"
#include "route_mgr.hxx"


FGRouteMgr::FGRouteMgr() :
    active( new SGRoute ),
    standby( new SGRoute ),
    last_lon( 0.0 ),
    last_lat( 0.0 ),
    last_az( 0.0 ),
    // start_mode( FIRST_WPT ),
    // follow_mode( XTRACK_LEG_HDG ),
    // completion_mode( LOOP ),
    dist_remaining_m( 0.0 )
{
    bind();
}


FGRouteMgr::~FGRouteMgr() {
    delete standby;
    delete active;
}


// bind property nodes
void FGRouteMgr::bind() {
    // property nodes
    pos_node = pyGetNode("/position", true);
    vel_node = pyGetNode("/velocity", true);
    orient_node = pyGetNode("/orientation", true);
    route_node = pyGetNode("/task/route", true);
    home_node = pyGetNode("/task/home", true);
    L1_node = pyGetNode("/config/autopilot/L1_controller", true);
    targets_node = pyGetNode("/autopilot/targets", true);

    // sanity check, set some conservative values if none are provided
    // in the autopilot config
    if ( L1_node.getDouble( "bank_limit_deg" ) < 0.1 ) {
	L1_node.setDouble( "bank_limit_deg", 20.0 );
    }
    if ( L1_node.getDouble( "period" ) < 0.1 ) {
	L1_node.setDouble( "period", 25.0 );
    }
    if ( L1_node.getDouble("damping") < 0.1 ) {
	L1_node.setDouble( "damping", 0.7 );
    }

    // defaults
    route_node.setString("follow_mode", "leader");
    route_node.setString("start_mode", "first_wpt");
    route_node.setString("completion_mode", "loop");
}


void FGRouteMgr::init() {
    pyPropertyNode default_route = pyGetNode("/config/route", true);
    init( &default_route );
}

void FGRouteMgr::init( pyPropertyNode *config_node ) {
    active->clear();
    standby->clear();

    if ( config_node != NULL ) {
	if ( ! build(config_node) ) {
	    printf("Detected an internal inconsistency in the route\n");
	    printf(" configuration.  See earlier errors for details.\n" );
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

    double wp_agl_m = 0.0;
    double wp_msl_m = 0.0;

    string request = route_node.getString("route_request");
    if ( request.length() ) {
	string result = "";
	if ( build(request) ) {
	    swap();
	    reposition();
	    result = "success: " + request;
	} else {
	    result = "failed: " + request;
	}		
	route_node.setString("request_result", result.c_str());
	route_node.setString("route_request", "");
    }

    route_node.setLong("route_size", active->size());
    if ( active->size() > 0 ) {
	if ( GPS_age() < 10.0 ) {

	    // route start up logic: if start_mode == first_wpt then
	    // there is nothing to do, we simply continue to track wpt
	    // 0 if that is the current waypoint.  If start_mode ==
	    // "first_leg", then if we are tracking wpt 0, increment
	    // it so we track the 2nd waypoint along the first leg.
	    // If only a 1 point route is given along with first_leg
	    // startup behavior, then don't do that again, force some
	    // sort of sane route parameters instead!
	    string start_mode = route_node.getString("start_mode");
	    if ( (start_mode == "first_leg")
		 && (active->get_waypoint_index() == 0) ) {
		if ( active->size() > 1 ) {
		    active->increment_current();
		} else {
		    route_node.setString("start_mode", "first_wpt");
		    route_node.setString("follow_mode", "direct");
		}
	    }

	    double L1_period = L1_node.getDouble("period");
	    double L1_damping = L1_node.getDouble("damping");
	    double gs_mps = vel_node.getDouble("groundspeed_ms");

	    // track current waypoint of route (only if we have fresh gps data)
	    SGWayPoint prev = active->get_previous();
	    SGWayPoint wp = active->get_current();

	    // compute direct-to course and distance
	    wp.CourseAndDistance( pos_node.getDouble("longitude_deg"),
				  pos_node.getDouble("latitude_deg"),
				  pos_node.getDouble("altitude_ft"),
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
	    route_node.setDouble( "xtrack_dist_m", xtrack_m );
	    route_node.setDouble( "projected_dist_m", dist_m );

	    // compute cross-track steering compensation
	    double xtrack_gain = route_node.getDouble("xtrack_steer_gain");
	    double xtrack_comp = xtrack_m * xtrack_gain;
	    if ( xtrack_comp < -45.0 ) { xtrack_comp = -45.0; }
	    if ( xtrack_comp > 45.0 ) { xtrack_comp = 45.0; }

	    // default distance for waypoint acquisition = direct
	    // distance to the target waypoint.  This can be
	    // overridden later by leg following and replaced with
	    // distance remaining along the leg.
	    nav_dist_m = direct_distance;

	    string follow_mode = route_node.getString("follow_mode");
	    string completion_mode = route_node.getString("completion_mode");
	    if ( follow_mode == "direct" ) {
		// direct to
		nav_course = direct_course;
	    } else if ( follow_mode == "xtrack_direct_hdg" ) {
		// cross track steering
		if ( (active->get_waypoint_index() == 0)
		     && (completion_mode != "loop") ) {
		    // first waypoint is 'direct to' except for LOOP
		    // routes which track the leg connecting the last
		    // wpt to the first wpt.
		    nav_course = direct_course;
		} else if ( (active->get_waypoint_index() == active->size()-1)
			    && (completion_mode == "extend_last_leg") ) {
		    // force leg heading logic on last leg so it is
		    // possible to extend the center line beyond the
		    // waypoint (if requested by completion mode.)
		    nav_course = leg_course - xtrack_comp;
		} else if ( fabs(angle) <= 45.0 ) {
		    // normal case, note: in this tracking mode,
		    // xtrack_course based on direct_course could
		    // start oscillating the closer we get to the
		    // target waypoint.
		    nav_course = direct_course - xtrack_comp;
		} else {
		    // navigate 'direct to' if off by more than the
		    // xtrack system can compensate for
		    nav_course = direct_course;
		}
	    } else if ( follow_mode == "xtrack_leg_hdg" ) {
		// cross track steering
		if ( (active->get_waypoint_index() == 0)
		     && (completion_mode != "loop") ) {
		    // first waypoint is 'direct to' except for LOOP
		    // routes which track the leg connecting the last
		    // wpt to the first wpt.
		    nav_course = direct_course;
		} else if ( (active->get_waypoint_index() == active->size()-1)
			    && (completion_mode == "extend_last_leg") ) {
		    // force leg heading logic on last leg so it is
		    // possible to extend the center line beyond the
		    // waypoint (if requested by completion mode.)
		    nav_dist_m = dist_m;
		    nav_course = leg_course - xtrack_comp;
		} else {
		    // normal case: xtrack_course based on leg_course,
		    // will have consistent stable regardless of
		    // distance from waypoint.
		    nav_dist_m = dist_m;
		    nav_course = leg_course - xtrack_comp;
		}
	    } else if ( follow_mode == "leader" ) {
		double L1_dist = (1.0 / SGD_PI) * L1_damping * L1_period * gs_mps;
		double wangle = 0.0;
		if ( L1_dist < 0.01 ) {
		    // ground speed <= 0.0 (problem?!?)
		    nav_course = direct_course;
		} else if ( L1_dist <= fabs(xtrack_m) ) {
		    // beyond L1 distance, steer as directly toward
		    // leg as allowed
		    wangle = 0.0;
		} else {
		    // steer towards imaginary point projected onto
		    // the route leg L1_distance ahead of us
		    wangle = acos(fabs(xtrack_m) / L1_dist) * SGD_RADIANS_TO_DEGREES;
		}
		if ( wangle < 30.0 ) {
		    wangle = 30.0;
		}
		if ( xtrack_m > 0.0 ) {
		    nav_course = direct_course + angle - 90.0 + wangle;
		} else {
		    nav_course = direct_course + angle + 90.0 - wangle;
		}
		if ( active->is_acquired() ) {
		    nav_dist_m = dist_m;
		} else {
		    // direct to first waypoint until we've acquired this route
		    nav_course = direct_course;
		    nav_dist_m = direct_distance;
		}

		// printf("direct=%.1f angle=%.1f nav=%.1f L1=%.1f xtrack=%.1f wangle=%.1f nav_dist=%.1f\n", direct_course, angle, nav_course, L1_dist, xtrack_m, wangle, nav_dist_m);
	    }

            if ( nav_course < 0.0 ) {
                nav_course += 360.0;
            } else if ( nav_course > 360.0 ) {
                nav_course -= 360.0;
            }

	    targets_node.setDouble( "groundtrack_deg", nav_course );

	    // target bank angle computed here

	    double target_bank_deg = 0.0;

	    const double sqrt_of_2 = 1.41421356237309504880;
	    double omegaA = sqrt_of_2 * SGD_PI / L1_period;
	    double VomegaA = gs_mps * omegaA;
	    double course_error = orient_node.getDouble("groundtrack_deg")
		- targets_node.getDouble("groundtrack_deg");
	    if ( course_error < -180.0 ) { course_error += 360.0; }
	    if ( course_error >  180.0 ) { course_error -= 360.0; }

	    double accel = 2.0 * sin(course_error * SG_DEGREES_TO_RADIANS) * VomegaA;
	    // double accel = 2.0 * gs_mps * gs_mps * sin(course_error * SG_DEGREES_TO_RADIANS) / L1;

	    static const double gravity = 9.81; // m/sec^2
	    double target_bank = -atan( accel / gravity );
	    target_bank_deg = target_bank * SG_RADIANS_TO_DEGREES;

	    double bank_limit_deg = L1_node.getDouble("bank_limit_deg");
	    if ( target_bank_deg < -bank_limit_deg ) {
		target_bank_deg = -bank_limit_deg;
	    }
	    if ( target_bank_deg > bank_limit_deg ) {
		target_bank_deg = bank_limit_deg;
	    }
	    targets_node.setDouble( "roll_deg", target_bank_deg );

	    wp_agl_m = wp.get_target_agl_m();
	    wp_msl_m = wp.get_target_alt_m();

	    // estimate distance remaining to completion of route
	    dist_remaining_m = nav_dist_m
		+ active->get_remaining_distance_from_current_waypoint();
	    route_node.setDouble("dist_remaining_m", dist_remaining_m);

#if 0
	    if ( display_on ) {
		printf("next leg: %.1f  to end: %.1f  wpt=%d of %d\n",
		       nav_dist_m, dist_remaining_m,
		       active->get_waypoint_index(), active->size());
	    }
#endif

	    // logic to mark completion of leg and move to next leg.
	    if ( completion_mode == "loop" ) {
		if ( nav_dist_m < 50.0 ) {
		    active->set_acquired( true );
		    active->increment_current();
		}
	    } else if ( completion_mode == "circle_last_wpt" ) {
		if ( nav_dist_m < 50.0 ) {
		    active->set_acquired( true );
		    if ( active->get_waypoint_index() < active->size() - 1 ) {
			active->increment_current();
		    } else {
			SGWayPoint wp = active->get_current();
			// FIXME: NEED TO GO TO CIRCLE MODE HERE SOME HOW!!!
			/*mission_mgr.request_task_circle(wp.get_target_lon(),
							wp.get_target_lat(),
							0.0, 0.0);*/
		    }
		}
	    } else if ( completion_mode == "extend_last_leg" ) {
		if ( nav_dist_m < 50.0 ) {
		    active->set_acquired( true );
		    if ( active->get_waypoint_index() < active->size() - 1 ) {
			active->increment_current();
		    } else {
			// follow the last leg forever
		    }
		}
	    }

	    // publish current target waypoint
	    route_node.setLong( "target_waypoint_idx",
				active->get_waypoint_index() );

	    // if ( display_on ) {
	    // printf("route dist = %0f\n", dist_remaining_m);
	    // }
	}
    } else {
        // FIXME: we've been commanded to follow a route, but no route
        // has been defined.

        // We are in ill-defined territory, should we do some sort of
        // circle of our home position?

	// FIXME: need to go to circle mode somehow here!!!!
	/* mission_mgr.request_task_circle(); */
    }

    route_node.setDouble( "wp_dist_m", direct_distance );

    // update target altitude based on waypoint target altitudes if
    // specified.  Preference is given to agl if both agl & msl are
    // set.
    if ( wp_agl_m > 1.0 ) {
	targets_node.setDouble( "altitude_agl_ft", wp_agl_m * SG_METER_TO_FEET );
    } else if ( wp_msl_m > 1.0 ) {
	targets_node.setDouble( "target_msl_ft", wp_msl_m * SG_METER_TO_FEET );
    }

    double gs_mps = vel_node.getDouble("groundspeed_ms");
    if ( gs_mps > 0.1 ) {
	route_node.setDouble( "wp_eta_sec", direct_distance / gs_mps );
    } else {
	route_node.setDouble( "wp_eta_sec", 0.0 );
    }
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


// build a route from a property (sub) tree
bool FGRouteMgr::build( pyPropertyNode *config_node ) {
    standby->clear();
    vector <string> children = config_node->getChildren();
    unsigned int count = children.size();
    for ( unsigned int i = 0; i < count; ++i ) {
        string name = children[i];
	pyPropertyNode child = config_node->getChild(name.c_str());
        // cout << name << endl;
        if ( name.substr(0,3) == "wpt" ) {
            SGWayPoint wpt( &child );
            standby->add_waypoint( wpt );
	} else if ( name == "enable" ) {
	    // happily ignore this (FIXME? Do we need this tag here?)
        } else {
            printf("Unknown top level section: %s\n", name.c_str() );
            return false;
        }
    }
    printf("loaded %d waypoints\n", standby->size());
    return true;
}


// build a route from a string request
bool FGRouteMgr::build( string request ) {
    vector <string> token = split( request, "," );
    if ( token.size() < 5 ) {
	return false;
    } else if ( route_mgr == NULL ) {
	return false;
    }
    standby->clear();
    unsigned int i = 1;
    while ( i + 4 <= token.size() ) {
	int mode = atoi( token[i].c_str() );
	double field1 = atof( token[i+1].c_str() );
	double field2 = atof( token[i+2].c_str() );
	double agl_m = -9999.9;
	if ( token[i+3] != "-" ) {
	    agl_m = atof( token[i+3].c_str() ) * SG_FEET_TO_METER;
	}
	new_waypoint( field1, field2, agl_m, mode );
	i += 4;
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


bool FGRouteMgr::reposition() {
    double home_lon = home_node.getDouble("longitude_deg");
    double home_lat = home_node.getDouble("latitude_deg");
    double home_az = home_node.getDouble("azimuth_deg");

    SGWayPoint wp(home_lon, home_lat);
    return reposition_pattern(wp, home_az);
}


bool FGRouteMgr::reposition_if_necessary() {
    double home_lon = home_node.getDouble("longitude_deg");
    double home_lat = home_node.getDouble("latitude_deg");
    double home_az = home_node.getDouble("azimuth_deg");

    if ( fabs(home_lon - last_lon) > 0.000001 ||
	 fabs(home_lat - last_lat) > 0.000001 ||
	 fabs(home_az - last_az) > 0.001 )
    {
	reposition();

	last_lon = home_lon;
	last_lat = home_lat;
	last_az = home_az;

	return true;
    }

    return false;
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
