/**
 * \file: circle_mgr.hxx
 *
 * Configure autopilot settings to fly to a specified coordinate
 *
 * Copyright (C) 2015 - Curtis L. Olson curtolson@flightgear.org
 *
 */


#ifndef _AURA_CIRCLE_MGR_HXX
#define _AURA_CIRCLE_MGR_HXX

#include "python/pyprops.hxx"

#include "control/waypoint.hxx"
#include "util/timing.h"


class AuraCircleMgr {

private:

    // automatic inputs
    pyPropertyNode lon_node;
    pyPropertyNode lat_node;
    pyPropertyNode alt_agl_node;
    pyPropertyNode true_heading_node;
    pyPropertyNode groundtrack_node;
    pyPropertyNode groundspeed_node;

    // configuration nodes
    pyPropertyNode coord_lon_node;
    pyPropertyNode coord_lat_node;
    pyPropertyNode direction_node;
    pyPropertyNode radius_node;
    pyPropertyNode target_agl_node;
    pyPropertyNode target_speed_node;
    pyPropertyNode bank_limit_node;
    pyPropertyNode L1_period_node;

    // autopilot settings
    pyPropertyNode fcs_mode_node;
    pyPropertyNode ap_agl_node;
    pyPropertyNode ap_roll_node;
    pyPropertyNode target_course_deg;

    pyPropertyNode wp_dist_m;
    pyPropertyNode wp_eta_sec;

    bool bind();		// setup property node bindings

public:

    AuraCircleMgr();
    ~AuraCircleMgr();

    bool init();	// code to run when task is created
    bool update();	// code to runs while task is active

    SGWayPoint get_center();	// return circle center

    void set_direction( const string direction );
    void set_radius( const double radius_m );
};


#endif // _AURA_CIRCLE_MGR_HXX
