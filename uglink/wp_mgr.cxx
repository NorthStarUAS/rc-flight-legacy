#include <stdio.h>

/* #include <simgear/math/polar3d.hxx> */
#include <math/SGMath.hxx>

#include "wp_mgr.hxx"


UGWaypoint::UGWaypoint():
    dirty(true),
    current_wp(0),
    wp_lon(0.0),
    wp_lat(0.0),
    cur_lon(0.0),
    cur_lat(0.0),
    cur_vn(0.0),
    cur_ve(0.0),
    dist_m(0.0),
    last_dist_m(0.0),
    eta_sec(0.0),
    timestamp(0.0),
    last_time(0),
    progress_mps(0.0)
{
}


UGWaypoint::~UGWaypoint() {
}


void UGWaypoint::update_math() {
    // compute distance from current position to target waypoint
    SGGeoc current = SGGeoc::fromDegM( cur_lon, cur_lat, 0.0 );
    SGGeoc target = SGGeoc::fromDegM( wp_lon, wp_lat, 0.0 );
    dist_m = SGGeoc::distanceM( current, target );

    // compute ground track speed
    double ground_speed_mps = sqrt( cur_vn*cur_vn + cur_ve*cur_ve );

    // compute rate of progress towards waypoint
    double dt = timestamp - last_time;
    last_time = timestamp;
    if ( dt > 0.0 ) {
	if ( dt > 1.0 ) { dt = 1.0; /* keep a sane value */ }
	double progress_m = last_dist_m - dist_m;
	last_dist_m = dist_m;
	double mps = progress_m / dt;
	if ( progress_m > 0.0 ) {
	    progress_mps = 0.99 * progress_mps + 0.01 * mps;
	}
    }

    // printf("gs = %.1f  pr = %.1f\n", ground_speed_mps, progress_mps);

    if ( progress_mps > 0.1 ) {
	eta_sec = dist_m / progress_mps;
    } else if ( ground_speed_mps > 0.1 ) {
	eta_sec = dist_m / ground_speed_mps;
    }

    dirty = false;
}


// update target waypoint when current_wp == wp_index
bool UGWaypoint::update_wp( uint16_t _current_wp, uint16_t _wp_index,
			    double _wp_lon, double _wp_lat )
{
    if ( _current_wp == _wp_index ) {
	current_wp = _current_wp;
	wp_lon = _wp_lon;
	wp_lat = _wp_lat;

	dirty = true;
    }

    return true;
}

// update position and velocity vector
bool UGWaypoint::update_pos_vel( double _timestamp,
				 double _cur_lon, double _cur_lat,
				 double _cur_vn, double _cur_ve )
{
    timestamp = _timestamp;
    cur_lon = _cur_lon;
    cur_lat = _cur_lat;
    cur_vn = _cur_vn;
    cur_ve = _cur_ve;

    dirty = true;

    return true;
}


double UGWaypoint::get_dist_m() {
    if ( dirty ) {
	update_math();
    }

    return dist_m;
}

string UGWaypoint::get_dist_str() {
    if ( dirty ) {
	update_math();
    }

    char buf[256];
    double dist_nm = dist_m * SG_METER_TO_NM;
    if ( dist_nm >= 10.0 ) {
	snprintf( buf, 128, "%.1fnm", dist_nm );
    } else if ( dist_nm >= 1.0 ) {
	snprintf( buf, 128, "%.2fnm", dist_nm );
    } else {
	snprintf( buf, 128, "%.0fm", dist_m );
    }
    return (string)buf;
}


double UGWaypoint::get_eta_sec() {
    if ( dirty ) {
	update_math();
    }

    return eta_sec;
}

string UGWaypoint::get_eta_str() {
    if ( dirty ) {
	update_math();
    }

    char buf[256];
    double tmp = eta_sec;
    int hours = (int)(eta_sec/3600.0);
    tmp -= hours*3600.0;
    int mins = (int)(tmp/60.0);
    tmp -= mins*60.0;
    double secs = tmp;
    if ( hours >= 1 ) {
	snprintf( buf, 128, "%d:%02d:%02.0f", hours, mins, secs );
    } else if ( mins >= 1 ) {
	snprintf( buf, 128, "%d:%02.0f", mins, secs );
    } else {
	snprintf( buf, 128, "%04.1f", secs );
    }
    return (string)buf;
}


UGWaypoint wp_mgr;
