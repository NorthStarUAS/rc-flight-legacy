// route_mgr.hxx - manage a route (i.e. a collection of waypoints)
//
// Written by Curtis Olson, started January 2004.
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
// $Id: route_mgr.hxx,v 1.11 2008/11/23 04:02:17 curt Exp $


#ifndef _ROUTE_MGR_HXX
#define _ROUTE_MGR_HXX

#ifndef __cplusplus
# error This library requires C++
#endif

#include <string>
#include <vector>

using std::string;
using std::vector;

#include "props/props.hxx"
#include "route.hxx"


/**
 * Top level route manager class
 * 
 */

class FGRouteMgr {

public:

    enum fgRouteMode {
        GoHome = 0,
        FollowRoute = 1
    };

private:

    SGRoute *route;
    SGWayPoint home;
    double home_course_deg;

    // route configuration
    SGPropertyNode_ptr config_props;

    // automatic inputs
    SGPropertyNode_ptr lon_node;
    SGPropertyNode_ptr lat_node;
    SGPropertyNode_ptr alt_node;
    SGPropertyNode_ptr vn_node;
    SGPropertyNode_ptr ve_node;

    // automatic outputs
    SGPropertyNode_ptr true_hdg_deg;
    SGPropertyNode_ptr target_agl_ft;
    SGPropertyNode_ptr override_agl_ft;
    SGPropertyNode_ptr target_msl_ft;
    SGPropertyNode_ptr override_msl_ft;
    SGPropertyNode_ptr target_waypoint;

    // route following mode
    SGPropertyNode *route_mode_node;

    // register "home" in the property tree
    SGPropertyNode *home_lon_node;
    SGPropertyNode *home_lat_node;

    // wind related property nodes
    SGPropertyNode *est_wind_speed_kt;
    SGPropertyNode *est_wind_dir_deg;
    SGPropertyNode *est_wind_east_mps;
    SGPropertyNode *est_wind_north_mps;
    SGPropertyNode *est_wind_true_heading_deg;
    SGPropertyNode *est_wind_target_heading_deg;

    // console/logging property nodes
    SGPropertyNode *ap_console_skip;
    SGPropertyNode *ap_logging_skip;

    bool home_set;
    bool msl_override;
    bool agl_override;

    fgRouteMode mode;

    SGWayPoint make_waypoint( const string& target );

    bool build();

public:

    FGRouteMgr();
    ~FGRouteMgr();

    void bind();

    void init();

    void update();

    int new_waypoint( const string& tgt_alt );
    void add_waypoint( const SGWayPoint& wp );
    void replace_waypoint( const SGWayPoint &wp, int n );

    SGWayPoint get_waypoint( int i ) const {
        return route->get_waypoint(i);
    }

    SGWayPoint pop_waypoint( int i = 0 );

    int size() const {
        return route->size();
    }

    bool update_home( const SGWayPoint &wp, const double hdg,
		      bool force_update );

    SGWayPoint get_home();

    void set_route_mode();

    void set_home_mode();
 
    inline fgRouteMode get_route_mode() {
        return mode;
    }
};


extern FGRouteMgr route_mgr;           // global route manager object


#endif // _ROUTE_MGR_HXX
