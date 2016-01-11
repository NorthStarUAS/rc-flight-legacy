// umngnss_euler.cpp -- C++/Property aware interface for GNSS/ADNS 15-state
//                      kalman filter algorithm
//


#include "python/pyprops.hxx"

#include <math.h>
#include <string.h>

#include "include/globaldefs.h"
#include "sensors/gps_mgr.hxx"

#include "umngnss_euler.h"
#include "adns.h"


// property nodes
static pyPropertyNode imu_node;
static pyPropertyNode gps_node;
static pyPropertyNode filter_node;


int umngnss_euler_init( string rootname, pyPropertyNode *config ) {
    // initialize imu property nodes
    imu_node = pyGetNode("/sensors/imu");
    gps_node = pyGetNode("/sensors/gps");
    filter_node = pyGetNode(rootname);

    filter_node.setString( "navigation", "invalid" );
    int result = umn_adns_init();

    return result;
}


bool umngnss_euler_update() {
    static bool umn_init_pos = false;

    bool fresh_data = false;

    if ( GPS_age() < 1.0 && gps_node.getBool("settle") && !umn_init_pos )
    {
	umn_init_pos = true;
	NavState s;
	memset( &s, 0, sizeof(NavState) );
	s.pos[0] = gps_node.getDouble("latitude_deg") * SGD_DEGREES_TO_RADIANS;
	s.pos[1] = gps_node.getDouble("longitude_deg") * SGD_DEGREES_TO_RADIANS;
	s.pos[2] = -gps_node.getDouble("altitude_m");
	s.eul[0] = SGD_PI_2 - atan2( gps_node.getDouble("vn_ms"),
				     gps_node.getDouble("ve_ms") );
	umn_adns_set_initial_state( &s );
	umn_adns_print_state( &s );
    }	    

    if ( umn_init_pos ) {
	double imu[7], gps[7];
	imu[0] = imu_node.getDouble("timestamp");
	imu[1] = imu_node.getDouble("p_rad_sec");
	imu[2] = imu_node.getDouble("q_rad_sec");
	imu[3] = imu_node.getDouble("r_rad_sec");
	imu[4] = imu_node.getDouble("ax_mps_sec");
	imu[5] = imu_node.getDouble("ay_mps_sec");
	imu[6] = imu_node.getDouble("az_mps_sec");
	gps[0] = gps_node.getDouble("timestamp");
	gps[1] = gps_node.getDouble("latitude_deg") * SGD_DEGREES_TO_RADIANS;
	gps[2] = gps_node.getDouble("longitude_deg") * SGD_DEGREES_TO_RADIANS;
	gps[3] = -gps_node.getDouble("altitude_m");
	gps[4] = gps_node.getDouble("vn_ms");
	gps[5] = gps_node.getDouble("ve_ms");
	gps[6] = gps_node.getDouble("vd_ms");
	// umn_adns_print_gps( gps );
	umn_adns_update( imu, gps );

	NavState *s = umn_adns_get_state();

	// publish values to property tree
	double psi = s->eul[0];
	if ( psi < 0 ) { psi += SGD_2PI; }
	if ( psi > SGD_2PI ) { psi -= SGD_2PI; }
	filter_node.setDouble( "timestamp", imu[0] );
	filter_node.setDouble( "roll_deg", s->eul[2] * SG_RADIANS_TO_DEGREES );
	filter_node.setDouble( "pitch_deg", s->eul[1] * SG_RADIANS_TO_DEGREES );
	filter_node.setDouble( "heading_deg", psi * SG_RADIANS_TO_DEGREES );
	filter_node.setDouble( "latitude_deg", s->pos[0] * SG_RADIANS_TO_DEGREES );
	filter_node.setDouble( "longitude_deg", s->pos[1] * SG_RADIANS_TO_DEGREES );
	filter_node.setDouble( "altitude_m", -s->pos[2] );
	filter_node.setDouble( "vn_ms", s->vel[0] );
	filter_node.setDouble( "ve_ms", s->vel[1] );
	filter_node.setDouble( "vd_ms", s->vel[2] );
	filter_node.setString( "navigation", "valid");

	filter_node.setDouble( "altitude_ft", -s->pos[2] * SG_METER_TO_FEET );
	filter_node.setDouble( "groundtrack_deg", 90 - atan2(s->vel[0], s->vel[1])
	 				* SG_RADIANS_TO_DEGREES );
	filter_node.setDouble( "groundspeed_ms", sqrt( s->vel[0] * s->vel[0]
	 				    + s->vel[1] * s->vel[1] ) );
        filter_node.setDouble( "vertical_speed_fps",
			       -s->vel[2] * SG_METER_TO_FEET );

	fresh_data = true;
    }

    return fresh_data;
}


int umngnss_euler_close() {
    return umn_adns_close();
}

