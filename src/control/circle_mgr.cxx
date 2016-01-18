/**
 * \file: task_circle_coord.cxx
 *
 * Task: configure autopilot settings to fly to a circle around a specified
 * point.  Compensate circle track using wind estimate to try to achieve a 
 * better circle form.
 *
 * Copyright (C) 2011 - Curtis L. Olson curtolson@flightgear.org
 *
 */

#include "python/pyprops.hxx"

#include <cstdio>
#include <cmath>

#include "comms/display.hxx"
#include "control/waypoint.hxx"
#include "include/globaldefs.h"
#include "util/wind.hxx"

#include "circle_mgr.hxx"

AuraCircleMgr::AuraCircleMgr() {
    bind();
};


AuraCircleMgr::~AuraCircleMgr() {
};


bool AuraCircleMgr::bind() {
    // property nodes
    pos_node = pyGetNode("/position", true);
    vel_node = pyGetNode("velocity", true);
    orient_node = pyGetNode("/orientation", true);
    circle_node = pyGetNode("/task/circle", true);
    L1_node = pyGetNode("/config/fcs/autopilot/L1_controller", true);
    fcs_node = pyGetNode("/config/fcs", true);
    ap_node = pyGetNode("/autopilot/settings", true);
    route_node = pyGetNode("/task/route", true);

    // sanity check, set some conservative values if none are provided
    // in the autopilot config
    if ( L1_node.getDouble( "bank_limit_deg" ) < 0.1 ) {
	L1_node.setDouble( "bank_limit_deg", 20.0 );
    }
    if ( L1_node.getDouble( "period" ) < 0.1 ) {
	L1_node.setDouble( "period", 25.0 );
    }

    return true;
}


// FIXME: make sure we have sane default values setup some where
// Maybe: setup home as the default circle location if not otherwise set?
// How does this play with the mission system?  We don't want to do
// nothing and have the aircraft just fly off in a straight line forever.
bool AuraCircleMgr::init() {

    return true;
}


bool AuraCircleMgr::update() {
    // printf("circle update\n");

    string direction_str = circle_node.getString("direction");
    double direction = 1.0;
    if ( direction_str == "right" ) {
	direction = -1.0;
    }

    SGWayPoint target = SGWayPoint( circle_node.getDouble("longitude_deg"),
				    circle_node.getDouble("latitude_deg") );

    // compute course and distance to center of target circle
    double course_deg;
    double dist_m;
    target.CourseAndDistance( pos_node.getDouble("longitude_deg"),
			      pos_node.getDouble("latitude_deg"),
			      0.0, &course_deg, &dist_m );

    // compute ideal ground course to be on the circle perimeter if at
    // ideal radius
    double ideal_crs = course_deg + direction * 90;
    if ( ideal_crs > 360.0 ) { ideal_crs -= 360.0; }
    if ( ideal_crs < 0.0 ) { ideal_crs += 360.0; }

    // (in)sanity check
    double radius_m = circle_node.getDouble("radius_m");
    if ( radius_m < 25.0 ) { radius_m = 25.0; }

    // compute a target ground course based on our actual radius distance
    double target_crs = ideal_crs;
    if ( dist_m < radius_m ) {
	// inside circle, adjust target heading to expand our circling
	// radius
	double offset_deg = direction * 90.0 * (1.0 - dist_m / radius_m);
	target_crs += offset_deg;
	if ( target_crs > 360.0 ) { target_crs -= 360.0; }
	if ( target_crs < 0.0 ) { target_crs += 360.0; }
    } else if ( dist_m > radius_m ) {
	// outside circle, adjust target heading to tighten our
	// circling radius
	double offset_dist = dist_m - radius_m;
	if ( offset_dist > radius_m ) { offset_dist = radius_m; }
	double offset_deg = direction * 90 * offset_dist / radius_m;
	target_crs -= offset_deg;
	if ( target_crs > 360.0 ) { target_crs -= 360.0; }
	if ( target_crs < 0.0 ) { target_crs += 360.0; }
    }
    ap_node.setDouble( "target_groundtrack_deg", target_crs );
    /*if ( display_on ) {
	printf("rad=%.0f act=%.0f ideal crs=%.1f tgt crs=%.1f\n",
	       radius_m, dist_m, ideal_crs, target_crs);
      }*/

    // new L1 'mathematical' response to error

    double L1_period = L1_node.getDouble("period");	// gain
    double gs_mps = vel_node.getDouble("groundspeed_ms");
    const double sqrt_of_2 = 1.41421356237309504880;
    double omegaA = sqrt_of_2 * SGD_PI / L1_period;
    double VomegaA = gs_mps * omegaA;
    double course_error = orient_node.getDouble("groundtrack_deg") - target_crs;
    if ( course_error < -180.0 ) { course_error += 360.0; }
    if ( course_error >  180.0 ) { course_error -= 360.0; }
    // accel: is the lateral acceleration we need to compensate for
    // heading error
    double accel = 2.0 * sin(course_error * SG_DEGREES_TO_RADIANS) * VomegaA;
    // double accel = 2.0 * gs_mps * gs_mps * sin(course_error * SG_DEGREES_TO_RADIANS) / L1;

    // ideal_accel: the steady state lateral accel we would expect
    // when we are in the groove exactly on our target radius
    double ideal_accel = direction * gs_mps * gs_mps / radius_m;

    // circling acceleration needed for our current distance from center
    double turn_accel = direction * gs_mps * gs_mps / dist_m;

    // old way over turns when tracking inbound from a long distance
    // away
    // double total_accel = accel + ideal_accel;
    
    // compute desired acceleration = acceleration required for course
    // correction + acceleration required to maintain turn at current
    // distance from center.
    double total_accel = accel + turn_accel;

    static const double gravity = 9.81; // m/sec^2
    double target_bank = -atan( total_accel / gravity );
    double target_bank_deg = target_bank * SG_RADIANS_TO_DEGREES;

    double bank_limit_deg = L1_node.getDouble("bank_limit_deg");
    if ( target_bank_deg < -bank_limit_deg ) {
	target_bank_deg = -bank_limit_deg;
    }
    if ( target_bank_deg > bank_limit_deg ) {
	target_bank_deg = bank_limit_deg;
    }
    //printf("   circle: tgt bank = %.0f  bank limit = %.0f\n",
    //	   target_bank_deg, bank_limit_deg);

    ap_node.setDouble( "target_roll_deg", target_bank_deg );

    // printf("circle: ground_crs = %.1f aircraft_hdg = %.1f\n",
    //	   course_deg, hd_deg );

    route_node.setDouble( "wp_dist_m", dist_m );
    if ( gs_mps > 0.1 ) {
	route_node.setDouble( "wp_eta_sec", dist_m / gs_mps );
    } else {
	route_node.setDouble( "wp_eta_sec", 0.0 );
    }

    return true;
};


SGWayPoint AuraCircleMgr::get_center() {
    return SGWayPoint( circle_node.getDouble("longitude_deg"),
		       circle_node.getDouble("latitude_deg") );
}


void AuraCircleMgr::set_direction( const string direction ) {
    circle_node.setString( "direction", direction.c_str() );
}

void AuraCircleMgr::set_radius( const double radius_m ) {
    circle_node.setDouble( "radius_m", radius_m );
}
