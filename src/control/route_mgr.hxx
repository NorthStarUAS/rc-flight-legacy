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

    enum StartMode {
	FIRST_WPT = 0,		// Go to first waypoint
	FIRST_LEG = 1,		// Go to 2nd waypoint along route leg
    };

#if 0
    enum FollowMode {
	DIRECT = 0,		// steer direct to next waypoint
	XTRACK_LEG_HDG = 1,	// steer towards leg heading + xtrack
	XTRACK_DIRECT_HDG = 2,	// steer direct oto next wpt + xtrack
	LEADER = 3		// steer towards a projected lead point
    };
#endif
    
    enum CompletionMode {
	LOOP = 0,		// loop the route when finished
	CIRCLE_LAST_WPT = 1,	// circle the final waypoint in the route
	EXTEND_LAST_LEG = 2	// track the last route leg indefinitely
	// idea: reverse and fly backwards to home
	// idea: swap to standby route and fly that
	// idea: return home and circle
	// idea: rally points
    };

private:

    SGRoute *active;
    SGRoute *standby;

    pyPropertyNode pos_node;
    pyPropertyNode vel_node;
    pyPropertyNode orient_node;
    pyPropertyNode route_node;
    pyPropertyNode L1_node;
    pyPropertyNode ap_node;
    pyPropertyNode home_node;
    
    double last_lon;
    double last_lat;
    double last_az;

    // route behaviors
    StartMode start_mode;
    // FollowMode follow_mode;
    CompletionMode completion_mode;

    // stats
    double dist_remaining_m;

    SGWayPoint make_waypoint( const string& wpt_string );

    bool build( pyPropertyNode *config_node );

public:

    FGRouteMgr();
    ~FGRouteMgr();

    void bind();

    void init();
    void init( pyPropertyNode *config_node );

    // set route start mode
    inline void set_start_mode( enum StartMode mode ) {
	start_mode = mode;
    }

#if 0
    // set route follow mode
    inline void set_follow_mode( enum FollowMode mode ) {
	follow_mode = mode;
    }
#endif

    // set route completion mode
    inline void set_completion_mode( enum CompletionMode mode ) {
	completion_mode = mode;
    }

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
