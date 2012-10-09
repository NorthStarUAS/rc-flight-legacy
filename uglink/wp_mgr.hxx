#ifndef _UGLINK_WP_MGR_HXX
#define _UGLINK_WP_MGR_HXX

// track waypoint data sent from vehicle and compute time to next waypoint


//#include <simgear/compiler.h>
//#include <simgear/constants.h>

#include <string>

#include <stdint.h>

using std::string;


// Manage UGear Waypoints
class UGWaypoint {

private:

    bool dirty;

    uint16_t current_wp;
    double wp_lon;
    double wp_lat;

    double cur_lon;
    double cur_lat;
    double cur_vn;
    double cur_ve;

    double dist_m;
    double last_dist_m;
    double eta_sec;

    double timestamp;
    double last_time;
    double progress_mps;

    void update_math();

public:

    UGWaypoint();
    ~UGWaypoint();

    // update target waypoint when current_wp == wp_index
    bool update_wp( uint16_t _current_wp, uint16_t _wp_index,
		   double _wp_lon, double _wp_lat );

    // update position and velocity vector
    bool update_pos_vel( double _timestamp, double _cur_lon, double _cur_lat,
			 double _cur_vn, double _cur_ve );

    double get_dist_m();
    string get_dist_str();

    double get_eta_sec();
    string get_eta_str();

};


extern UGWaypoint wp_mgr;


#endif // _UGLINK_WP_MGR_HXX
