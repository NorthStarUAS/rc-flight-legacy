/**
 * \file: imu_mgr.cpp
 *
 * Front end management interface for reading IMU data.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@gmail.com
 *
 * $Id: imu_mgr.cpp,v 1.2 2009/04/30 14:39:39 curt Exp $
 */


#include <math.h>
#include <string.h>

#include "globaldefs.h"

#include "adns/mnav/ahrs.h"
#include "comms/console_link.h"
#include "comms/logging.h"
#include "props/props.hxx"
#include "util/myprof.h"

#include "mnav.h"
#include "ugfile.h"

#include "imu_mgr.h"

//
// Global variables
//

static imu_source_t source = imuNone;

// imu property nodes
static SGPropertyNode *imu_source_node = NULL;

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


void IMU_init() {
    // initialize imu property nodes
    timestamp_node = fgGetNode("/sensors/imu/timestamp", true);
    p_node = fgGetNode("/sensors/imu/p-radps", true);
    q_node = fgGetNode("/sensors/imu/q-radps", true);
    r_node = fgGetNode("/sensors/imu/r-radps", true);
    ax_node = fgGetNode("/sensors/imu/ax-mpss", true);
    ay_node = fgGetNode("/sensors/imu/ay-mpss", true);
    az_node = fgGetNode("/sensors/imu/az-mpss", true);
    hx_node = fgGetNode("/sensors/imu/hx", true);
    hy_node = fgGetNode("/sensors/imu/hy", true);
    hz_node = fgGetNode("/sensors/imu/hz", true);

    imu_source_node = fgGetNode("/config/sensors/imu-source", true);
    if ( strcmp(imu_source_node->getStringValue(), "mnav") == 0 ) {
	source = imuMNAV;
    } else if ( strcmp(imu_source_node->getStringValue(), "file") == 0 ) {
	source = imuUGFile;
    }

    // initialize property tree nodes (example)
    // _lat_node = fgGetNode("/position/latitude-gps-deg", true);

    switch ( source ) {
    case imuMNAV:
	// Initialize the communcation channel with the MNAV
 	mnav_init();
	break;
    case imuUGFile:
	// Initialize the communcation channel with the MNAV
 	ugfile_init();
	break;
    default:
	if ( display_on ) {
	    printf("Warning (init): no imu source defined\n");
	}
    }

}


bool IMU_update() {
    struct imu imupacket;
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

    case imuUGFile:
        ugfile_read();

	fresh_data = ugfile_get_imu(&imupacket);

	break;

    default:
	if ( display_on ) {
	    printf("Warning (update): no imu source defined\n");
	}
    }

    if ( fresh_data ) {
	// publish values to property tree
	timestamp_node->setDoubleValue( imupacket.time );
	p_node->setDoubleValue( imupacket.p );
	q_node->setDoubleValue( imupacket.q );
	r_node->setDoubleValue( imupacket.r );
	ax_node->setDoubleValue( imupacket.ax );
	ay_node->setDoubleValue( imupacket.ay );
	az_node->setDoubleValue( imupacket.az );
	hx_node->setDoubleValue( imupacket.hx );
	hy_node->setDoubleValue( imupacket.hy );
	hz_node->setDoubleValue( imupacket.hz );

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

    case imuUGFile:
	ugfile_close();
	break;

    default:
	if ( display_on ) {
	    printf("Warning: no imu source defined\n");
	}
    }
}
