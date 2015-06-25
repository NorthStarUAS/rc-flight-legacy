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

#include "props/props.hxx"
#include "util/timing.h"


class AuraCircleMgr {

private:

    // automatic inputs
    SGPropertyNode *lon_node;
    SGPropertyNode *lat_node;
    SGPropertyNode *alt_agl_node;
    SGPropertyNode *true_heading_node;
    SGPropertyNode *groundtrack_node;
    SGPropertyNode *groundspeed_node;

    // configuration nodes
    SGPropertyNode *coord_lon_node;
    SGPropertyNode *coord_lat_node;
    SGPropertyNode *direction_node;
    SGPropertyNode *radius_node;
    SGPropertyNode *target_agl_node;
    SGPropertyNode *target_speed_node;
    SGPropertyNode *bank_limit_node;
    SGPropertyNode *L1_period_node;

    // advanced behavior nodes
    SGPropertyNode *exit_agl_node;
    SGPropertyNode *exit_heading_node;

    // autopilot settings
    SGPropertyNode *fcs_mode_node;
    SGPropertyNode *ap_speed_node;
    SGPropertyNode *ap_agl_node;
    SGPropertyNode *ap_roll_node;
    SGPropertyNode *target_course_deg;

    SGPropertyNode *wp_dist_m;
    SGPropertyNode *wp_eta_sec;

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
