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
// $Id: route_mgr.hxx,v 1.1 2007/08/06 19:41:48 curt Exp $


#ifndef _ROUTE_MGR_HXX
#define _ROUTE_MGR_HXX 1

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

private:

    SGRoute *route;

    // route configuration
    SGPropertyNode_ptr config_props;

    // automatic inputs
    SGPropertyNode_ptr lon;
    SGPropertyNode_ptr lat;
    SGPropertyNode_ptr alt;

    // automatic outputs
    SGPropertyNode_ptr true_hdg_deg;
    SGPropertyNode_ptr target_altitude_ft;

    bool altitude_set;

    SGWayPoint make_waypoint( const string& target );

public:

    FGRouteMgr();
    ~FGRouteMgr();

    void init ();
    bool build ();

    void update (double dt);

    int new_waypoint( const string& tgt_alt, int n = -1 );
    void add_waypoint( const SGWayPoint& wp, int n = -1 );
    SGWayPoint pop_waypoint( int i = 0 );

    SGWayPoint get_waypoint( int i ) const {
        return route->get_waypoint(i);
    }

    int size() const {
        return route->size();
    }

};


#endif // _ROUTE_MGR_HXX
