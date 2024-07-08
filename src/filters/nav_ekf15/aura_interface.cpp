// aura_interface.cpp -- C++/Property aware interface for GNSS/ADNS 15-state
//                       kalman filter algorithm
//


#include <math.h>
#include <string.h>

#include "include/globaldefs.h"

#include "../nav_common/constants.h"

#include "aura_interface.h"
#include "EKF_15state.h"

// these are the important sensor and result structures used by the
// UMN code.  To avoid pointers and dynamic allocation, create static
// copies of these and use them henceforth.

static EKF15 filter;

static IMUdata imu_data;
static GPSdata gps_data;
static NAVdata nav_data;

// property nodes
static PropertyNode imu_node;
static PropertyNode gps_node;
static PropertyNode filter_node;

// when false will trigger a nav init if gps is alive and settled
static bool nav_inited = false;

// update the imu_data and gps_data structures with most recent sensor
// data prior to calling the filter init or update routines
static void props2umn(void) {
    imu_data.time = imu_node.getDouble("timestamp");
    imu_data.p = imu_node.getDouble("p_rps");
    imu_data.q = imu_node.getDouble("q_rps");
    imu_data.r = imu_node.getDouble("r_rps");
    imu_data.ax = imu_node.getDouble("ax_mps2");
    imu_data.ay = imu_node.getDouble("ay_mps2");
    imu_data.az = imu_node.getDouble("az_mps2");
    imu_data.hx = imu_node.getDouble("hx");
    imu_data.hy = imu_node.getDouble("hy");
    imu_data.hz = imu_node.getDouble("hz");

    gps_data.time = gps_node.getDouble("timestamp");
    gps_data.lat = gps_node.getDouble("latitude_deg");
    gps_data.lon = gps_node.getDouble("longitude_deg");
    gps_data.alt = gps_node.getDouble("altitude_m");
    gps_data.vn = gps_node.getDouble("vn_mps");
    gps_data.ve = gps_node.getDouble("ve_mps");
    gps_data.vd = gps_node.getDouble("vd_mps");
}

// update the property tree values from the nav_data structure
// returned by the umn filter init or update routines
static void umn2props(void) {
    double psi = nav_data.psi;
    if ( psi < 0 ) { psi += M_PI*2.0; }
    if ( psi > M_PI*2.0 ) { psi -= M_PI*2.0; }
    filter_node.setDouble( "timestamp", imu_data.time );
    filter_node.setUInt( "millis", imu_data.time * 1000 );
    filter_node.setDouble( "roll_deg", nav_data.phi * R2D );
    filter_node.setDouble( "pitch_deg", nav_data.the * R2D );
    filter_node.setDouble( "yaw_deg", psi * R2D );
    filter_node.setDouble( "latitude_deg", nav_data.lat * R2D );
    filter_node.setDouble( "longitude_deg", nav_data.lon * R2D );
    filter_node.setDouble( "altitude_m", nav_data.alt );
    filter_node.setDouble( "vn_mps", nav_data.vn );
    filter_node.setDouble( "ve_mps", nav_data.ve );
    filter_node.setDouble( "vd_mps", nav_data.vd );
    if ( nav_data.err_type == data_valid ||
	 nav_data.err_type == TU_only ||
	 nav_data.err_type == gps_aided )
    {
	filter_node.setInt( "status", 2 );
    } else {
	filter_node.setInt( "status", 1 );
    }

    filter_node.setDouble( "p_bias", nav_data.gbx );
    filter_node.setDouble( "q_bias", nav_data.gby );
    filter_node.setDouble( "r_bias", nav_data.gbz );
    filter_node.setDouble( "ax_bias", nav_data.abx );
    filter_node.setDouble( "ay_bias", nav_data.aby );
    filter_node.setDouble( "az_bias", nav_data.abz );
    
    float max_pos_cov = nav_data.Pp0;
    if ( nav_data.Pp1 > max_pos_cov ) { max_pos_cov = nav_data.Pp1; }
    if ( nav_data.Pp2 > max_pos_cov ) { max_pos_cov = nav_data.Pp2; }
    if ( max_pos_cov > 655.0 ) { max_pos_cov = 655.0; }
    float max_vel_cov = nav_data.Pv0;
    if ( nav_data.Pv1 > max_vel_cov ) { max_vel_cov = nav_data.Pv1; }
    if ( nav_data.Pv2 > max_vel_cov ) { max_vel_cov = nav_data.Pv2; }
    if ( max_vel_cov > 65.5 ) { max_vel_cov = 65.5; }
    float max_att_cov = nav_data.Pa0;
    if ( nav_data.Pa1 > max_att_cov ) { max_att_cov = nav_data.Pa1; }
    if ( nav_data.Pa2 > max_att_cov ) { max_att_cov = nav_data.Pa2; }
    if ( max_att_cov > 6.55 ) { max_vel_cov = 6.55; }
    filter_node.setDouble( "max_pos_cov", max_pos_cov );
    filter_node.setDouble( "max_vel_cov", max_vel_cov );
    filter_node.setDouble( "max_att_cov", max_att_cov );
    
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


void nav_ekf15_init( string output_path, PropertyNode *config ) {
    // initialize property nodes
    imu_node = PropertyNode( "/sensors/imu/0" );
    gps_node = PropertyNode( "/sensors/gps/0" );
    filter_node = PropertyNode( output_path );
    filter_node.setInt( "status", 0 );

#if 0
    // set tuning value for specific gps and imu noise characteristics
    cov_gps_hpos_node = config.getChild("cov-gps-hpos");
    cov_gps_vpos_node = config.getChild("cov-gps-vpos");
    cov_gps_hvel_node = config.getChild("cov-gps-hvel");
    cov_gps_vvel_node = config.getChild("cov-gps-vvel");
    sigma_w_f_node = config.getChild("sigma-w-f");
    sigma_w_g_node = config.getChild("sigma-w-g");
    sigma_c_f_node = config.getChild("sigma-c-f");
    sigma_c_g_node = config.getChild("sigma-c-g");
    tau_f_node = config.getChild("tau-f");
    tau_g_node = config.getChild("tau-g");
#endif
}

// trigger an ekf reset
void nav_ekf15_reset() {
    nav_inited = false;
}

bool nav_ekf15_update() {
    static double last_gps_time = 0.0;

    // fill in the UMN structures
    props2umn();

    if ( nav_inited ) {
	filter.time_update( imu_data );
        if ( gps_data.time > last_gps_time ) {
            last_gps_time = gps_data.time;
            filter.measurement_update( gps_data );
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


void nav_ekf15_close() {
    // noop()
}

