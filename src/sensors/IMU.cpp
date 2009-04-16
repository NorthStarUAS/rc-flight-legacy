/**
 * \file: IMU.cpp
 *
 * Front end management interface for reading IMU data.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@gmail.com
 *
 * $Id: IMU.cpp,v 1.5 2009/04/16 20:35:46 curt Exp $
 */


#include <math.h>
#include <string.h>

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


bool IMU_update() {
    bool fresh_data = false;

    switch ( source ) {

    case imuMNAV:
	mnav_prof.start();
	// read IMU until no data available.  This will flush any
	// potential backlog that could accumulate for any reason.
	mnav_start_nonblock_read();
        while ( mnav_read_nonblock() );
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

    return fresh_data;
}


void IMU_close() {
    switch ( source ) {

    case imuMNAV:
	// nop
	break;

    default:
	if ( display_on ) {
	    printf("Warning: no gps source defined\n");
	}
    }
}
