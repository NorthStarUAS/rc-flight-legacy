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
    // property nodes
    pyPropertyNode pos_node;
    pyPropertyNode vel_node;
    pyPropertyNode orient_node;
    pyPropertyNode circle_node;
    pyPropertyNode L1_node;
    pyPropertyNode ap_node;
    pyPropertyNode route_node;

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
