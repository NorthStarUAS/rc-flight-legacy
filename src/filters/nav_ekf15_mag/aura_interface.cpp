// aura_interface.cpp -- C++/Property aware interface for GNSS/ADNS 15-state
//                       kalman filter algorithm
//


#include <pyprops.h>

#include <math.h>
#include <string.h>

#include "include/globaldefs.h"

#include "../nav_common/constants.h"

#include "aura_interface.h"
#include "EKF_15state.h"

// these are the important sensor and result structures used by the
// UMN code.  To avoid pointers and dynamic allocation, create static
// copies of these and use them henceforth.

static EKF15_mag filter;

static IMUdata imu_data;
static GPSdata gps_data;
static NAVdata nav_data;

// property nodes
static pyPropertyNode imu_node;
static pyPropertyNode gps_node;
static pyPropertyNode filter_node;

// when false will trigger a nav init if gps is alive and settled
static bool nav_inited = false;

// update the imu_data and gps_data structures with most recent sensor
// data prior to calling the filter init or update routines
static void props2umn(void) {
    imu_data.time = imu_node.getDouble("timestamp");
    imu_data.p = imu_node.getDouble("p_rad_sec");
    imu_data.q = imu_node.getDouble("q_rad_sec");
    imu_data.r = imu_node.getDouble("r_rad_sec");
    imu_data.ax = imu_node.getDouble("ax_mps_sec");
    imu_data.ay = imu_node.getDouble("ay_mps_sec");
    imu_data.az = imu_node.getDouble("az_mps_sec");
    imu_data.hx = imu_node.getDouble("hx");
    imu_data.hy = imu_node.getDouble("hy");
    imu_data.hz = imu_node.getDouble("hz");

    gps_data.time = gps_node.getDouble("timestamp");
    gps_data.lat = gps_node.getDouble("latitude_deg");
    gps_data.lon = gps_node.getDouble("longitude_deg");
    gps_data.alt = gps_node.getDouble("altitude_m");
    gps_data.vn = gps_node.getDouble("vn_ms");
    gps_data.ve = gps_node.getDouble("ve_ms");
    gps_data.vd = gps_node.getDouble("vd_ms");
}

// update the property tree values from the nav_data structure
// returned by the umn filter init or update routines
static void umn2props(void) {
    double psi = nav_data.psi;
    if ( psi < 0 ) { psi += M_PI*2.0; }
    if ( psi > M_PI*2.0 ) { psi -= M_PI*2.0; }
    filter_node.setDouble( "timestamp", imu_data.time );
    filter_node.setDouble( "roll_deg", nav_data.phi * R2D );
    filter_node.setDouble( "pitch_deg", nav_data.the * R2D );
    filter_node.setDouble( "heading_deg", psi * R2D );
    filter_node.setDouble( "latitude_deg", nav_data.lat * R2D );
    filter_node.setDouble( "longitude_deg", nav_data.lon * R2D );
    filter_node.setDouble( "altitude_m", nav_data.alt );
    filter_node.setDouble( "vn_ms", nav_data.vn );
    filter_node.setDouble( "ve_ms", nav_data.ve );
    filter_node.setDouble( "vd_ms", nav_data.vd );
    if ( nav_data.err_type == data_valid ||
	 nav_data.err_type == TU_only ||
	 nav_data.err_type == gps_aided )
    {
	filter_node.setString( "navigation", "valid" );
    } else {
	filter_node.setString( "navigation", "invalid" );
    }

    filter_node.setDouble( "p_bias", nav_data.gbx );
    filter_node.setDouble( "q_bias", nav_data.gby );
    filter_node.setDouble( "r_bias", nav_data.gbz );
    filter_node.setDouble( "ax_bias", nav_data.abx );
    filter_node.setDouble( "ay_bias", nav_data.aby );
    filter_node.setDouble( "az_bias", nav_data.abz );
    
    filter_node.setDouble( "altitude_ft",
			   nav_data.alt * M2F );
    filter_node.setDouble( "groundtrack_deg",
			   90 - atan2(nav_data.vn, nav_data.ve) * R2D );
    double gs_ms = sqrt(nav_data.vn * nav_data.vn + nav_data.ve * nav_data.ve);
    filter_node.setDouble( "groundspeed_ms", gs_ms );
    filter_node.setDouble( "groundspeed_kt", gs_ms * SG_MPS_TO_KT );
    filter_node.setDouble( "vertical_speed_fps",
			   -nav_data.vd * M2F );
}


void nav_ekf15_mag_init( string output_path, pyPropertyNode *config ) {
    // initialize property nodes
    imu_node = pyGetNode("/sensors/imu", true);
    gps_node = pyGetNode("/sensors/gps", true);
    filter_node = pyGetNode(output_path, true);
    filter_node.setString( "navigation", "invalid" );

#if 0
    // set tuning value for specific gps and imu noise characteristics
    cov_gps_hpos_node = config.getChild("cov-gps-hpos", 0, true);
    cov_gps_vpos_node = config.getChild("cov-gps-vpos", 0, true);
    cov_gps_hvel_node = config.getChild("cov-gps-hvel", 0, true);
    cov_gps_vvel_node = config.getChild("cov-gps-vvel", 0, true);
    sigma_w_f_node = config.getChild("sigma-w-f", 0, true);
    sigma_w_g_node = config.getChild("sigma-w-g", 0, true);
    sigma_c_f_node = config.getChild("sigma-c-f", 0, true);
    sigma_c_g_node = config.getChild("sigma-c-g", 0, true);
    tau_f_node = config.getChild("tau-f", 0, true);
    tau_g_node = config.getChild("tau-g", 0, true);
#endif
}


// trigger an ekf reset
void nav_ekf15_mag_reset() {
    nav_inited = false;
}


bool nav_ekf15_mag_update() {
    static double last_gps_time = 0.0;

    // fill in the UMN structures
    props2umn();

    if ( nav_inited ) {
	filter.time_update( imu_data );
        if ( gps_data.time > last_gps_time ) {
            last_gps_time = gps_data.time;
            filter.measurement_update( imu_data, gps_data );
        }
        nav_data = filter.get_nav();
    } else {
	if ( gps_node.getDouble("data_age") < 1.0 && gps_node.getBool("settle") ) {
	    filter.init( imu_data, gps_data );
            nav_data = filter.get_nav();
	    nav_inited = true;
	}
    }

    // copy the nav_data results back to the property tree
    umn2props();

    return nav_inited;
}


void nav_ekf15_mag_close() {
    // noop()
}

