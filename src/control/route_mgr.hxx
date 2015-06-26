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

    enum StartMode {
	FIRST_WPT = 0,		// Go to first waypoint
	FIRST_LEG = 1,		// Go to 2nd waypoint along route leg
    };

    enum FollowMode {
	DIRECT = 0,		// steer direct to next waypoint
	XTRACK_LEG_HDG = 1,	// steer towards leg heading + xtrack
	XTRACK_DIRECT_HDG = 2,	// steer direct oto next wpt + xtrack
	LEADER = 3		// steer towards a projected lead point
    };

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

    // route configuration tree
    SGPropertyNode *config_props;

    // home
    SGPropertyNode *home_lon_node;
    SGPropertyNode *home_lat_node;
    SGPropertyNode *home_azimuth_node;

    double last_lon;
    double last_lat;
    double last_az;

    // route following configuration
    SGPropertyNode *bank_limit_node;
    SGPropertyNode *L1_period_node;
    SGPropertyNode *L1_damping_node;
    SGPropertyNode *xtrack_gain_node;

    // automatic inputs
    SGPropertyNode *lon_node;
    SGPropertyNode *lat_node;
    SGPropertyNode *alt_node;
    SGPropertyNode *groundspeed_node;
    SGPropertyNode *groundtrack_node;

    // automatic outputs
    SGPropertyNode *target_heading_error_deg;
    SGPropertyNode *target_course_deg;
    SGPropertyNode *ap_roll_node;
    SGPropertyNode *target_agl_node;
    SGPropertyNode *target_msl_node;
    SGPropertyNode *target_waypoint;
    SGPropertyNode *wp_dist_m;
    SGPropertyNode *wp_eta_sec;
    SGPropertyNode *xtrack_dist_m;
    SGPropertyNode *proj_dist_m;

    // route behaviors
    StartMode start_mode;
    FollowMode follow_mode;
    CompletionMode completion_mode;

    // stats
    double dist_remaining_m;

    SGWayPoint make_waypoint( const string& wpt_string );

    bool build();

public:

    FGRouteMgr();
    ~FGRouteMgr();

    void bind();

    void init( SGPropertyNode *branch );

    // set route start mode
    inline void set_start_mode( enum StartMode mode ) {
	start_mode = mode;
    }

    // set route follow mode
    inline void set_follow_mode( enum FollowMode mode ) {
	follow_mode = mode;
    }

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
