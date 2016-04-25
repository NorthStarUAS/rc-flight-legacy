// route_mgr.hxx - manage a route (i.e. a collection of waypoints)
//
// Written by Curtis Olson, started January 2004.
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


#ifndef _ROUTE_MGR_HXX
#define _ROUTE_MGR_HXX

#ifndef __cplusplus
# error This library requires C++
#endif

#include "python/pyprops.hxx"

#include <string>
#include <vector>
using std::string;
using std::vector;

#include "route.hxx"


/**
 * Top level route manager class
 * 
 */

class FGRouteMgr {

public:

private:

    SGRoute *active;
    SGRoute *standby;

    pyPropertyNode pos_node;
    pyPropertyNode vel_node;
    pyPropertyNode orient_node;
    pyPropertyNode route_node;
    pyPropertyNode L1_node;
    pyPropertyNode targets_node;
    pyPropertyNode home_node;
    
    double last_lon;
    double last_lat;
    double last_az;

    // route behaviors
    // StartMode start_mode;
    // FollowMode follow_mode;
    // CompletionMode completion_mode;

    // stats
    double dist_remaining_m;

    SGWayPoint make_waypoint( const string& wpt_string );

    // build a route from a property (sub) tree
    bool build( pyPropertyNode *config_node );
    
    // build a route from a string request
    bool build( string request );

public:

    FGRouteMgr();
    ~FGRouteMgr();

    void bind();

    void init();
    void init( pyPropertyNode *config_node );

    void update();

    // swap the "active" and the "standby" routes, but only if the
    // "standby" route has some waypoints.
    bool swap();

    // these modify the "standby" route
    inline void clear_standby() {
	standby->clear();
    }
    int new_waypoint( const string& wpt_string );
    int new_waypoint( const double lon, const double lat, const double alt,
		      const int mode );

    // returns info on the "active" route
    inline SGWayPoint get_waypoint( int i ) const {
        return active->get_waypoint(i);
    }
    inline int get_waypoint_index() const {
	return active->get_waypoint_index();
    }
    inline int size() const {
        return active->size();
    }
    inline double get_dist_remaining_m() const {
	return dist_remaining_m;
    }

    // applies to the "active" route
    bool reposition();
    bool reposition_if_necessary();
    bool reposition_pattern( const SGWayPoint &wp, const double hdg );

    // restart the route from the beginning
    inline void restart() {
	active->set_acquired( false );
	active->set_current( 0 );
    }
};


#endif // _ROUTE_MGR_HXX
