/**
 * \file: IMU.cpp
 *
 * Front end management interface for reading IMU data.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@gmail.com
 *
 * $Id: IMU.cpp,v 1.1 2009/01/17 20:28:01 curt Exp $
 */


#include <math.h>

#include "globaldefs.h"

#include "comms/console_link.h"
#include "comms/logging.h"
#include "props/props.hxx"
#include "util/myprof.h"

#include "mnav.h"
#include "IMU.h"

//
// Global variables
//

// shared gps structure
struct imu imupacket;

static imu_source_t source = imuNone;

// imu property nodes
static SGPropertyNode *imu_source_node = NULL;


void IMU_init() {
    // initialize gps property nodes
    imu_source_node = fgGetNode("/config/sensors/imu-source", true);
    if ( strcmp(imu_source_node->getStringValue(), "mnav") == 0 ) {
	source = imuMNAV;
    }

    // gps_lat_node = fgGetNode("/position/latitude-gps-deg", true);

    switch ( source ) {
    case imuMNAV:
	// Initialize the communcation channel with the MNAV
 	mnav_init();
	break;
    default:
	if ( display_on ) {
	    printf("Warning: no imu source defined\n");
	}
    }

}

// NOTE: this is the master time syncronization routine in ugear.
// This routine should block until new IMU data is available.  The
// rate at which the IMU sends data dictates the timing and rate of
// the entire ugear program.  Currently the code expects MNAV-like
// behavior where the IMU is configured to send fresh data at a
// consistant 50hz pace.

void IMU_update() {
    bool fresh_data = false;

    switch ( source ) {

    case imuMNAV:
	mnav_prof.start();
        mnav_read();
	mnav_prof.stop();

	fresh_data = mnav_get_imu(&imupacket);

	break;

    default:
	if ( display_on ) {
	    printf("Warning: no imu source defined\n");
	}
    }

    if ( fresh_data ) {
	// publish values to property tree
	// gps_lat_node->setDoubleValue( gpspacket.lat );

	if ( console_link_on ) {
	    console_link_imu( &imupacket );
	}

	if ( log_to_file ) {
	    log_imu( &imupacket );
	}
    }
}


void IMU_close() {
    switch ( source ) {

    case imuMNAV:
	// nopp
	break;

    default:
	if ( display_on ) {
	    printf("Warning: no gps source defined\n");
	}
    }
}
