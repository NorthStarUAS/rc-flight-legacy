/**
 * \file: adns_mgr.cpp
 *
 * Front end management interface for executing the available ADNS codes.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@gmail.com
 *
 * $Id: adns_mgr.cpp,v 1.4 2009/05/13 22:09:04 curt Exp $
 */


#include <math.h>
#include <string.h>

#include "globaldefs.h"

#include "adns/mnav/ahrs.h"
#include "adns/mnav/nav.h"
#include "adns/umn/adns.h"
// #include "comms/console_link.h"
// #include "comms/logging.h"
#include "props/props.hxx"
#include "sensors/imu_mgr.h"	// temporary until imupacket dependency is removed?
#include "sensors/gps_mgr.h"
// #include "util/coremag.h"
// #include "util/timing.h"

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

// gps property nodes
static SGPropertyNode *gps_time_stamp_node = NULL;
static SGPropertyNode *gps_lat_node = NULL;
static SGPropertyNode *gps_lon_node = NULL;
static SGPropertyNode *gps_alt_node = NULL;
static SGPropertyNode *gps_ve_node = NULL;
static SGPropertyNode *gps_vn_node = NULL;
static SGPropertyNode *gps_vd_node = NULL;

// output property nodes
static SGPropertyNode *theta_node = NULL;
static SGPropertyNode *phi_node = NULL;
static SGPropertyNode *psi_node = NULL;

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

    // initialize gps property nodes
    gps_time_stamp_node = fgGetNode("/sensors/gps/time-stamp", true);
    gps_lat_node = fgGetNode("/sensors/gps/latitude-deg", true);
    gps_lon_node = fgGetNode("/sensors/gps/longitude-deg", true);
    gps_alt_node = fgGetNode("/sensors/gps/altitude-m", true);
    gps_ve_node = fgGetNode("/sensors/gps/ve-ms", true);
    gps_vn_node = fgGetNode("/sensors/gps/vn-ms", true);
    gps_vd_node = fgGetNode("/sensors/gps/vd-ms", true);

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
		umn_adns_init( );
	    }
	}
    }

    // initialize output property nodes (after module initialization
    // so we know that the reference properties will exist
    if ( toplevel->nChildren() > 0 ) {
	theta_node = fgGetNode("/orientation/pitch-deg", true);
	phi_node = fgGetNode("/orientation/roll-deg", true);
	psi_node = fgGetNode("/orientation/heading-deg", true);

	theta_node->alias("/adns/filter[0]/pitch-deg");
	phi_node->alias("/adns/filter[0]/roll-deg");
	psi_node->alias("/adns/filter[0]/heading-deg");
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
		static bool umn_init_pos = false;
		if ( GPS_age() < 1 && !umn_init_pos ) {
		    umn_init_pos = true;
		    NavState s;
		    memset( &s, 0, sizeof(NavState) );
		    s.pos[0] =  gps_lat_node->getDoubleValue()
			* SGD_DEGREES_TO_RADIANS;
		    s.pos[1] =  gps_lon_node->getDoubleValue()
			* SGD_DEGREES_TO_RADIANS;
		    s.pos[2] = -gps_alt_node->getDoubleValue();
		    umn_adns_set_initial_state( &s );
		    umn_adns_print_state( &s );
		}	    
		if ( umn_init_pos ) {
		    double imu[7], gps[7];
		    imu[0] = imupacket.time;
		    imu[1] = imupacket.p;
		    imu[2] = imupacket.q;
		    imu[3] = imupacket.r;
		    imu[4] = imupacket.ax;
		    imu[5] = imupacket.ay;
		    imu[6] = imupacket.az;
		    gps[0] = gps_time_stamp_node->getDoubleValue();
		    gps[1] = gps_lat_node->getDoubleValue() * SGD_DEGREES_TO_RADIANS;
		    gps[2] = gps_lon_node->getDoubleValue() * SGD_DEGREES_TO_RADIANS;
		    gps[3] = -gps_alt_node->getDoubleValue();
		    gps[4] = gps_vn_node->getDoubleValue();
		    gps[5] = gps_ve_node->getDoubleValue();
		    gps[6] = gps_vd_node->getDoubleValue();
		    // umn_adns_print_gps( gps );
		    umn_adns_update( imu, gps );
		}
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
		umn_adns_close();
	    }
	}
    }
}
