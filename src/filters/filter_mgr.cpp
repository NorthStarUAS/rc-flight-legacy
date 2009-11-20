/**
 * \file: filter_mgr.cpp
 *
 * Front end management interface for executing the available filter codes.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson <at> gmail <dot> com
 *
 * $Id: adns_mgr.cpp,v 1.6 2009/05/15 17:04:56 curt Exp $
 */


#include <stdio.h>

#include "include/ugear_config.h"

#include "comms/console_link.h"
#include "comms/logging.h"
#include "filters/curt/adns_curt.hxx"
#ifdef ENABLE_MNAV_FILTER
#  include "filters/mnav/ahrs.h"
#  include "filters/mnav/nav.h"
#endif // ENABLE_MNAV_FILTER
#include "filters/umn_interface.h"
#include "include/globaldefs.h"
#include "main/globals.hxx"
#include "props/props.hxx"
#include "util/myprof.h"

#include "filter_mgr.h"


//
// Global variables
//

static double last_imu_time = 0.0;

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

// filter property nodes
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

// comm property nodes
static SGPropertyNode *filter_console_skip = NULL;
static SGPropertyNode *filter_logging_skip = NULL;


void Filter_init() {
    // initialize imu property nodes
    imu_timestamp_node = fgGetNode("/sensors/imu/timestamp", true);
    imu_p_node = fgGetNode("/sensors/imu/p-rad_sec", true);
    imu_q_node = fgGetNode("/sensors/imu/q-rad_sec", true);
    imu_r_node = fgGetNode("/sensors/imu/r-rad_sec", true);
    imu_ax_node = fgGetNode("/sensors/imu/ax-mps_sec", true);
    imu_ay_node = fgGetNode("/sensors/imu/ay-mps_sec", true);
    imu_az_node = fgGetNode("/sensors/imu/az-mps_sec", true);
    imu_hx_node = fgGetNode("/sensors/imu/hx", true);
    imu_hy_node = fgGetNode("/sensors/imu/hy", true);
    imu_hz_node = fgGetNode("/sensors/imu/hz", true);

    // initialize comm nodes
    filter_console_skip = fgGetNode("/config/console/filter-skip", true);
    filter_logging_skip = fgGetNode("/config/logging/filter-skip", true);

    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/filters", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "filter" ) {
	    string module = section->getChild("module")->getStringValue();
	    bool enabled = section->getChild("enable")->getBoolValue();
	    if ( !enabled ) {
		continue;
	    }
	    string basename = "/filters/";
	    basename += section->getDisplayName();
	    printf("i = %d  name = %s module = %s %s\n",
	    	   i, name.c_str(), module.c_str(), basename.c_str());
	    
	    if ( module == "curt" ) {
		curt_adns_init( basename );
#ifdef ENABLE_MNAV_FILTER
	    } else if ( module == "mnav" ) {
		// Initialize AHRS code.  Must be called before
		// ahrs_update() or ahrs_close()
		mnav_ahrs_init( basename, section );

		// Initialize the NAV code.  Must be called before
		// nav_update() or nav_close()
		mnav_nav_init( basename );
#endif // ENABLE_MNAV_FILTER
	    } else if ( module == "umn" ) {
		ugumn_adns_init( basename );
	    }
	}
    }

    // initialize output property nodes (after module initialization
    // so we know that the reference properties will exist
    filter_theta_node = fgGetNode("/orientation/pitch-deg", true);
    filter_phi_node = fgGetNode("/orientation/roll-deg", true);
    filter_psi_node = fgGetNode("/orientation/heading-deg", true);
    filter_lat_node = fgGetNode("/position/latitude-deg", true);
    filter_lon_node = fgGetNode("/position/longitude-deg", true);
    filter_alt_node = fgGetNode("/position/altitude-m", true);
    filter_vn_node = fgGetNode("/velocity/vn-ms", true);
    filter_ve_node = fgGetNode("/velocity/ve-ms", true);
    filter_vd_node = fgGetNode("/velocity/vd-ms", true);
    filter_status_node = fgGetNode("/health/navigation", true);

    filter_alt_feet_node = fgGetNode("/position/altitude-ft", true);
    filter_track_node = fgGetNode("/orientation/groundtrack-deg", true);
    filter_vel_node = fgGetNode("/velocity/groundspeed-ms", true);
    filter_vert_speed_fps_node = fgGetNode("/velocity/vertical-speed-fps", true);

    if ( toplevel->nChildren() > 0 ) {
	filter_theta_node->alias("/filters/filter[0]/pitch-deg");
	filter_phi_node->alias("/filters/filter[0]/roll-deg");
	filter_psi_node->alias("/filters/filter[0]/heading-deg");
	filter_lat_node->alias("/filters/filter[0]/latitude-deg");
	filter_lon_node->alias("/filters/filter[0]/longitude-deg");
	filter_alt_node->alias("/filters/filter[0]/altitude-m");
	filter_vn_node->alias("/filters/filter[0]/vn-ms");
	filter_ve_node->alias("/filters/filter[0]/ve-ms");
	filter_vd_node->alias("/filters/filter[0]/vd-ms");
	filter_status_node->alias("/filters/filter[0]/navigation");

	filter_alt_feet_node->alias("/filters/filter[0]/altitude-ft");
	filter_track_node->alias("/filters/filter[0]/groundtrack-deg");
	filter_vel_node->alias("/filters/filter[0]/groundspeed-ms");
	filter_vert_speed_fps_node->alias("/filters/filter[0]/vertical-speed-fps");
    }
}


bool Filter_update( bool fresh_imu_data ) {
    filter_prof.start();

    double imu_time = imu_timestamp_node->getDoubleValue();
    double imu_dt = imu_time - last_imu_time;
    if ( imu_dt > 1.0 ) { imu_dt = 0.02; } // sanity check

    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/filters", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "filter" ) {
	    string module = section->getChild("module")->getStringValue();
	    bool enabled = section->getChild("enable")->getBoolValue();
	    if ( !enabled ) {
		continue;
	    }
	    if ( module == "null" ) {
		// do nothing
	    } else if ( module == "curt" ) {
		curt_adns_update( imu_dt );
#ifdef ENABLE_MNAV_FILTER
	    } else if ( module == "mnav" ) {
		static int mnav_nav_counter = 0;

		mnav_nav_counter++;
		struct imu imupacket;
		imupacket.time = imu_time;
		imupacket.p = imu_p_node->getDoubleValue();
		imupacket.q = imu_q_node->getDoubleValue();
		imupacket.r = imu_r_node->getDoubleValue();
		imupacket.ax = imu_ax_node->getDoubleValue();
		imupacket.ay = imu_ay_node->getDoubleValue();
		imupacket.az = imu_az_node->getDoubleValue();
		imupacket.hx = imu_hx_node->getDoubleValue();
		imupacket.hy = imu_hy_node->getDoubleValue();
		imupacket.hz = imu_hz_node->getDoubleValue();

		if ( fresh_imu_data ) {
		    // Run the MNAV AHRS algorithm.
		    mnav_ahrs_update( &imupacket );
		}
		if ( mnav_nav_counter >= 2 ) {
		    // navigation at half the rate of ahrs (assumes ahrs
		    // update @ 50hz and nav update @ 25hz.)  Compute a
		    // location estimate based on gps and accelerometer
		    // data.
		    mnav_nav_counter = 0;
		    mnav_nav_update( &imupacket );
		}
#endif // ENABLE_MNAV_FILTER
	    } else if ( module == "umn" ) {
		ugumn_adns_update();
	    }
	}
    }

    filter_prof.stop();

    if ( console_link_on || log_to_file ) {
	uint8_t buf[256];
	int size = packetizer->packetize_filter( buf );

        if ( console_link_on ) {
            console_link_filter( buf, size,
				 filter_console_skip->getIntValue() );
        }

        if ( log_to_file ) {
            log_filter( buf, size, filter_logging_skip->getIntValue() );
        }
    }

    last_imu_time = imu_time;

    return true;
}


void Filter_close() {
    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/filters", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "filter" ) {
	    string module = section->getChild("module")->getStringValue();
	    if ( module == "null" ) {
		// do nothing
#ifdef ENABLE_MNAV_FILTER
	    } else if ( module == "mnav" ) {
		mnav_ahrs_close();
		mnav_nav_close();
#endif // ENABLE_MNAV_FILTER
	    } else if ( module == "umn" ) {
		ugumn_adns_close();
	    }
	}
    }
}
