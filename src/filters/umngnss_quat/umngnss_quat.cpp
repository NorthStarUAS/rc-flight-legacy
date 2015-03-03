// umngnss_quat.cpp -- C++/Property aware interface for GNSS/ADNS 15-state
//                     kalman filter algorithm
//


#include <math.h>
#include <string.h>

#include "include/globaldefs.h"
#include "props/props.hxx"
#include "sensors/gps_mgr.hxx"

#include "umngnss_quat.h"
#include "nav_interface.h"
#include "globaldefs.h"

// these are the important sensor and result structures used by the
// UMN code.  To avoid pointers and dynamic allocation, create static
// copies of these and use them henceforth.
static struct imu imu_data;
static struct gps gps_data;
static struct nav nav_data;

// imu property nodes
static SGPropertyNode *imu_timestamp_node = NULL;
static SGPropertyNode *imu_p_node = NULL;
static SGPropertyNode *imu_q_node = NULL;
static SGPropertyNode *imu_r_node = NULL;
static SGPropertyNode *imu_ax_node = NULL;
static SGPropertyNode *imu_ay_node = NULL;
static SGPropertyNode *imu_az_node = NULL;
static SGPropertyNode *imu_hx_node = NULL;
static SGPropertyNode *imu_hy_node = NULL;
static SGPropertyNode *imu_hz_node = NULL;

// gps property nodes
static SGPropertyNode *gps_timestamp_node = NULL;
static SGPropertyNode *gps_lat_node = NULL;
static SGPropertyNode *gps_lon_node = NULL;
static SGPropertyNode *gps_alt_node = NULL;
static SGPropertyNode *gps_ve_node = NULL;
static SGPropertyNode *gps_vn_node = NULL;
static SGPropertyNode *gps_vd_node = NULL;
static SGPropertyNode *gps_settle_node = NULL;

// filter property nodes
static SGPropertyNode *filter_timestamp_node = NULL;
static SGPropertyNode *filter_theta_node = NULL;
static SGPropertyNode *filter_phi_node = NULL;
static SGPropertyNode *filter_psi_node = NULL;
static SGPropertyNode *filter_lat_node = NULL;
static SGPropertyNode *filter_lon_node = NULL;
static SGPropertyNode *filter_alt_node = NULL;
static SGPropertyNode *filter_vn_node = NULL;
static SGPropertyNode *filter_ve_node = NULL;
static SGPropertyNode *filter_vd_node = NULL;
static SGPropertyNode *filter_status_node = NULL;

static SGPropertyNode *filter_alt_feet_node = NULL;
static SGPropertyNode *filter_track_node = NULL;
static SGPropertyNode *filter_vel_node = NULL;
static SGPropertyNode *filter_vert_speed_fps_node = NULL;

#if 0
static SGPropertyNode *cov_gps_hpos_node = NULL;
static SGPropertyNode *cov_gps_vpos_node = NULL;
static SGPropertyNode *cov_gps_hvel_node = NULL;
static SGPropertyNode *cov_gps_vvel_node = NULL;
static SGPropertyNode *sigma_w_f_node = NULL;
static SGPropertyNode *sigma_w_g_node = NULL;
static SGPropertyNode *sigma_c_f_node = NULL;
static SGPropertyNode *sigma_c_g_node = NULL;
static SGPropertyNode *tau_f_node = NULL;
static SGPropertyNode *tau_g_node = NULL;
#endif


// update the imu_data and gps_data structures with most recent sensor
// data prior to calling the filter init or update routines
static void props2umn(void) {
    imu_data.time = imu_timestamp_node->getDoubleValue();
    imu_data.p = imu_p_node->getDoubleValue();
    imu_data.q = imu_q_node->getDoubleValue();
    imu_data.r = imu_r_node->getDoubleValue();
    imu_data.ax = imu_ax_node->getDoubleValue();
    imu_data.ay = imu_ay_node->getDoubleValue();
    imu_data.az = imu_az_node->getDoubleValue();
    
    gps_data.time = gps_timestamp_node->getDoubleValue();
    gps_data.lat = gps_lat_node->getDoubleValue();
    gps_data.lon = gps_lon_node->getDoubleValue();
    gps_data.alt = gps_alt_node->getDoubleValue();
    gps_data.vn = gps_vn_node->getDoubleValue();
    gps_data.ve = gps_ve_node->getDoubleValue();
    gps_data.vd = gps_vd_node->getDoubleValue();
    gps_data.newData = 0; /* FIXME */
}

// update the property tree values from the nav_data structure
// returned by the umn filter init or update routines
static void umn2props(void) {
    double psi = nav_data.psi;
    if ( psi < 0 ) { psi += SGD_2PI; }
    if ( psi > SGD_2PI ) { psi -= SGD_2PI; }
    filter_timestamp_node->setDoubleValue( imu_data.time );
    filter_phi_node->setDoubleValue( nav_data.phi * SG_RADIANS_TO_DEGREES );
    filter_theta_node->setDoubleValue( nav_data.the * SG_RADIANS_TO_DEGREES );
    filter_psi_node->setDoubleValue( psi * SG_RADIANS_TO_DEGREES );
    filter_lat_node->setDoubleValue( nav_data.lat * SG_RADIANS_TO_DEGREES );
    filter_lon_node->setDoubleValue( nav_data.lon * SG_RADIANS_TO_DEGREES );
    filter_alt_node->setDoubleValue( nav_data.alt );
    filter_vn_node->setDoubleValue( nav_data.vn );
    filter_ve_node->setDoubleValue( nav_data.ve );
    filter_vd_node->setDoubleValue( nav_data.vd );
    filter_status_node->setStringValue("valid"); /* FIXME */

    filter_alt_feet_node->setDoubleValue( nav_data.alt * SG_METER_TO_FEET );
    filter_track_node->setDoubleValue( 90 - atan2(nav_data.vn, nav_data.ve)
				       * SG_RADIANS_TO_DEGREES );
    filter_vel_node->setDoubleValue( sqrt(nav_data.vn * nav_data.vn
					  + nav_data.ve * nav_data.ve) );
    filter_vert_speed_fps_node
	->setDoubleValue( -nav_data.vd * SG_METER_TO_FEET );
}


void umngnss_quat_init( string rootname, SGPropertyNode *config ) {
    // initialize imu property nodes
    imu_timestamp_node = fgGetNode("/sensors/imu/time-stamp");
    imu_p_node = fgGetNode("/sensors/imu/p-rad_sec", true);
    imu_q_node = fgGetNode("/sensors/imu/q-rad_sec", true);
    imu_r_node = fgGetNode("/sensors/imu/r-rad_sec", true);
    imu_ax_node = fgGetNode("/sensors/imu/ax-mps_sec", true);
    imu_ay_node = fgGetNode("/sensors/imu/ay-mps_sec", true);
    imu_az_node = fgGetNode("/sensors/imu/az-mps_sec", true);
    imu_hx_node = fgGetNode("/sensors/imu/hx", true);
    imu_hy_node = fgGetNode("/sensors/imu/hy", true);
    imu_hz_node = fgGetNode("/sensors/imu/hz", true);

    // initialize gps property nodes
    gps_timestamp_node = fgGetNode("/sensors/gps/time-stamp", true);
    gps_lat_node = fgGetNode("/sensors/gps/latitude-deg", true);
    gps_lon_node = fgGetNode("/sensors/gps/longitude-deg", true);
    gps_alt_node = fgGetNode("/sensors/gps/altitude-m", true);
    gps_ve_node = fgGetNode("/sensors/gps/ve-ms", true);
    gps_vn_node = fgGetNode("/sensors/gps/vn-ms", true);
    gps_vd_node = fgGetNode("/sensors/gps/vd-ms", true);
    gps_settle_node = fgGetNode("/sensors/gps/settle", true);

    // initialize ahrs property nodes 
    SGPropertyNode *outputroot = fgGetNode( rootname.c_str(), true );
    filter_timestamp_node = outputroot->getChild("time-stamp", 0, true);
    filter_theta_node = outputroot->getChild("pitch-deg", 0, true);
    filter_phi_node = outputroot->getChild("roll-deg", 0, true);
    filter_psi_node = outputroot->getChild("heading-deg", 0, true);
    filter_lat_node = outputroot->getChild("latitude-deg", 0, true);
    filter_lon_node = outputroot->getChild("longitude-deg", 0, true);
    filter_alt_node = outputroot->getChild("altitude-m", 0, true);
    filter_vn_node = outputroot->getChild("vn-ms", 0, true);
    filter_ve_node = outputroot->getChild("ve-ms", 0, true);
    filter_vd_node = outputroot->getChild("vd-ms", 0, true);
    filter_status_node = outputroot->getChild("navigation",0, true);
    filter_status_node->setStringValue("invalid");

    filter_alt_feet_node = outputroot->getChild("altitude-ft", 0, true);
    filter_track_node = outputroot->getChild("groundtrack-deg", 0, true);
    filter_vel_node = outputroot->getChild("groundspeed-ms", 0, true);
    filter_vert_speed_fps_node
        = outputroot->getChild("vertical-speed-fps", 0, true);

#if 0
    // set tuning value for specific gps and imu noise characteristics
    cov_gps_hpos_node = config->getChild("cov-gps-hpos", 0, true);
    cov_gps_vpos_node = config->getChild("cov-gps-vpos", 0, true);
    cov_gps_hvel_node = config->getChild("cov-gps-hvel", 0, true);
    cov_gps_vvel_node = config->getChild("cov-gps-vvel", 0, true);
    sigma_w_f_node = config->getChild("sigma-w-f", 0, true);
    sigma_w_g_node = config->getChild("sigma-w-g", 0, true);
    sigma_c_f_node = config->getChild("sigma-c-f", 0, true);
    sigma_c_g_node = config->getChild("sigma-c-g", 0, true);
    tau_f_node = config->getChild("tau-f", 0, true);
    tau_g_node = config->getChild("tau-g", 0, true);
#endif
}


bool umngnss_quat_update() {
    static bool umn_inited = false;

    // fill in the UMN structures
    props2umn();

    if ( umn_inited ) {
	get_nav( &imu_data, &gps_data, &nav_data );
    } else {
	if ( GPS_age() < 1.0 && gps_settle_node->getBoolValue() ) {
	    init_nav( &imu_data, &gps_data, &nav_data );
	    umn_inited = true;
	}
    }

    // copy the nav_data results back to the property tree
    umn2props();

    return umn_inited;
}


void umngnss_quat_close() {
    close_nav();
}

