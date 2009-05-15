/**
 * \file: adns_mgr.cpp
 *
 * Front end management interface for executing the available ADNS codes.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@gmail.com
 *
 * $Id: adns_mgr.cpp,v 1.6 2009/05/15 17:04:56 curt Exp $
 */


#include "globaldefs.h"

#include "adns/mnav/ahrs.h"
#include "adns/mnav/nav.h"
#include "adns/umn_interface.h"
#include "props/props.hxx"

#include "adns_mgr.h"

//
// Global variables
//

// imu property nodes
static SGPropertyNode *timestamp_node = NULL;
static SGPropertyNode *p_node = NULL;
static SGPropertyNode *q_node = NULL;
static SGPropertyNode *r_node = NULL;
static SGPropertyNode *ax_node = NULL;
static SGPropertyNode *ay_node = NULL;
static SGPropertyNode *az_node = NULL;
static SGPropertyNode *hx_node = NULL;
static SGPropertyNode *hy_node = NULL;
static SGPropertyNode *hz_node = NULL;

// output property nodes
static SGPropertyNode *theta_node = NULL;
static SGPropertyNode *phi_node = NULL;
static SGPropertyNode *psi_node = NULL;

static SGPropertyNode *nav_status_node = NULL;
static SGPropertyNode *nav_lat_node = NULL;
static SGPropertyNode *nav_lon_node = NULL;
static SGPropertyNode *nav_alt_node = NULL;
static SGPropertyNode *nav_alt_feet_node = NULL;
static SGPropertyNode *nav_vn_node = NULL;
static SGPropertyNode *nav_ve_node = NULL;
static SGPropertyNode *nav_vd_node = NULL;
static SGPropertyNode *nav_track_node = NULL;
static SGPropertyNode *nav_vel_node = NULL;
static SGPropertyNode *nav_vert_speed_fps_node = NULL;

void ADNS_init() {
    // initialize imu property nodes
    timestamp_node = fgGetNode("/sensors/imu/timestamp", true);
    p_node = fgGetNode("/sensors/imu/p-rad_sec", true);
    q_node = fgGetNode("/sensors/imu/q-rad_sec", true);
    r_node = fgGetNode("/sensors/imu/r-rad_sec", true);
    ax_node = fgGetNode("/sensors/imu/ax-mps_sec", true);
    ay_node = fgGetNode("/sensors/imu/ay-mps_sec", true);
    az_node = fgGetNode("/sensors/imu/az-mps_sec", true);
    hx_node = fgGetNode("/sensors/imu/hx", true);
    hy_node = fgGetNode("/sensors/imu/hy", true);
    hz_node = fgGetNode("/sensors/imu/hz", true);

    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/filters", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "filter" ) {
	    string module = section->getChild("module")->getStringValue();
	    string basename = "/adns/";
	    basename += section->getDisplayName();
	    printf("i = %d  name = %s module = %s %s\n",
	    	   i, name.c_str(), module.c_str(), basename.c_str());
	    
	    if ( module == "mnav" ) {
		// Initialize AHRS code.  Must be called before
		// ahrs_update() or ahrs_close()
		mnav_ahrs_init( basename, section );

		// Initialize the NAV code.  Must be called before
		// nav_update() or nav_close()
		mnav_nav_init( basename );
	    } else if ( module == "umn" ) {
		ugumn_adns_init( basename );
	    }
	}
    }

    // initialize output property nodes (after module initialization
    // so we know that the reference properties will exist
    theta_node = fgGetNode("/orientation/pitch-deg", true);
    phi_node = fgGetNode("/orientation/roll-deg", true);
    psi_node = fgGetNode("/orientation/heading-deg", true);

    nav_status_node = fgGetNode("/health/navigation", true);
    nav_lat_node = fgGetNode("/position/latitude-deg", true);
    nav_lon_node = fgGetNode("/position/longitude-deg", true);
    nav_alt_node = fgGetNode("/position/altitude-m", true);
    nav_alt_feet_node = fgGetNode("/position/altitude-ft", true);
    nav_vn_node = fgGetNode("/velocity/vn-ms", true);
    nav_ve_node = fgGetNode("/velocity/ve-ms", true);
    nav_vd_node = fgGetNode("/velocity/vd-ms", true);
    nav_track_node = fgGetNode("/orientation/groundtrack-deg", true);
    nav_vel_node = fgGetNode("/velocity/groundspeed-ms", true);
    nav_vert_speed_fps_node = fgGetNode("/velocity/vertical-speed-fps", true);

    if ( toplevel->nChildren() > 0 ) {
	theta_node->alias("/adns/filter[0]/pitch-deg");
	phi_node->alias("/adns/filter[0]/roll-deg");
	psi_node->alias("/adns/filter[0]/heading-deg");

	nav_status_node->alias("/adns/filter[0]/navigation");
	nav_lat_node->alias("/adns/filter[0]/latitude-deg");
	nav_lon_node->alias("/adns/filter[0]/longitude-deg");
	nav_alt_node->alias("/adns/filter[0]/altitude-m");
	nav_alt_feet_node->alias("/adns/filter[0]/altitude-ft");
	nav_vn_node->alias("/adns/filter[0]/vn-ms");
	nav_ve_node->alias("/adns/filter[0]/ve-ms");
	nav_vd_node->alias("/adns/filter[0]/vd-ms");
	nav_track_node->alias("/adns/filter[0]/groundtrack-deg");
	nav_vel_node->alias("/adns/filter[0]/groundspeed-ms");
	nav_vert_speed_fps_node->alias("/adns/filter[0]/vertical-speed-fps");
    }
}


bool ADNS_update( bool fresh_imu_data ) {
    struct imu imupacket;
    imupacket.time = timestamp_node->getDoubleValue();
    imupacket.p = p_node->getDoubleValue();
    imupacket.q = q_node->getDoubleValue();
    imupacket.r = r_node->getDoubleValue();
    imupacket.ax = ax_node->getDoubleValue();
    imupacket.ay = ay_node->getDoubleValue();
    imupacket.az = az_node->getDoubleValue();
    imupacket.hx = hx_node->getDoubleValue();
    imupacket.hy = hy_node->getDoubleValue();
    imupacket.hz = hz_node->getDoubleValue();

    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/filters", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "filter" ) {
	    string module = section->getChild("module")->getStringValue();
	    if ( module == "mnav" ) {
		static int mnav_nav_counter = 0;
		mnav_nav_counter++;
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
	    } else if ( module == "umn" ) {
		ugumn_adns_update( &imupacket );
	    }
	}
    }

    return true;
}


void ADNS_close() {
    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/adns", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "filter" ) {
	    string module = section->getChild("module")->getStringValue();
	    if ( module == "mnav" ) {
		mnav_ahrs_close();
		mnav_nav_close();
	    } else if ( module == "umn" ) {
		ugumn_adns_close();
	    }
	}
    }
}
