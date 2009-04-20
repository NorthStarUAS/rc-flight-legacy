/**
 * \file: gps_mgr.cpp
 *
 * Front end management interface for reading GPS data.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@gmail.com
 *
 * $Id: gps_mgr.cpp,v 1.2 2009/04/20 02:14:59 curt Exp $
 */


#include <math.h>
#include <string.h>

#include "globaldefs.h"

#include "comms/console_link.h"
#include "comms/logging.h"
#include "props/props.hxx"

#include "gpsd.h"
#include "mnav.h"
#include "ugfile.h"

#include "gps_mgr.h"

//
// Global variables
//

// shared gps structure
struct gps gpspacket;

static gps_source_t source = gpsNone;

// gps property nodes
static SGPropertyNode *gps_source_node = NULL;

static SGPropertyNode *gps_lat_node = NULL;
static SGPropertyNode *gps_lon_node = NULL;
// static SGPropertyNode *gps_alt_node = NULL;
// static SGPropertyNode *gps_ve_node = NULL;
// static SGPropertyNode *gps_vn_node = NULL;
// static SGPropertyNode *gps_vd_node = NULL;
static SGPropertyNode *gps_track_node = NULL;
static SGPropertyNode *gps_unix_sec_node = NULL;


void GPS_init() {
    // initialize gps property nodes
    gps_source_node = fgGetNode("/config/sensors/gps-source", true);
    if ( strcmp(gps_source_node->getStringValue(), "file") == 0 ) {
	source = gpsUGFile;
    } else if ( strcmp(gps_source_node->getStringValue(), "gpsd") == 0 ) {
	source = gpsGPSD;
    } else if ( strcmp(gps_source_node->getStringValue(), "mnav") == 0 ) {
	source = gpsMNAV;
    }

    gps_lat_node = fgGetNode("/position/latitude-gps-deg", true);
    gps_lon_node = fgGetNode("/position/longitude-gps-deg", true);
    // gps_alt_node = fgGetNode("/position/altitude-gps-m", true);
    // gps_ve_node = fgGetNode("/velocities/ve-gps-ms", true);
    // gps_vn_node = fgGetNode("/velocities/vn-gps-ms", true);
    // gps_vd_node = fgGetNode("/velocities/vd-gps-ms", true);
    gps_track_node = fgGetNode("/orientation/groundtrack-gps-deg", true);
    gps_unix_sec_node = fgGetNode("/position/gps-unix-time-sec", true);

    // mnav conveys little useful date information from the gps,
    // fake it with a recent date that is close enough to compute
    // a reasonable magnetic variation, this should be updated
    // every year or two.
    gpspacket.date = 1232216597; /* Jan 17, 2009 */

    switch ( source ) {
    case gpsGPSD:
	gpsd_init();
	break;
    case gpsMNAV:
	// nothing to do
	break;
    case gpsUGFile:
	// nothing to do
	break;
    default:
	if ( display_on ) {
	    printf("Warning: no gps source defined\n");
	}
    }

}


bool GPS_update() {
    bool fresh_data = false;

    switch ( source ) {

    case gpsGPSD:
	fresh_data = gpsd_get_gps(&gpspacket);
	break;

    case gpsMNAV:
	fresh_data = mnav_get_gps(&gpspacket);

	break;

    case gpsUGFile:
	fresh_data = ugfile_get_gps(&gpspacket);
	break;

    default:
	if ( display_on ) {
	    printf("Warning: no gps source defined\n");
	}
    }

    if ( fresh_data ) {
	// publish values to property tree
	gps_lat_node->setDoubleValue( gpspacket.lat );
	gps_lon_node->setDoubleValue( gpspacket.lon );
	// gps_alt_node->setDoubleValue( gpspacket.alt );
	// gps_ve_node->setDoubleValue( gpspacket.ve );
	// gps_vn_node->setDoubleValue( gpspacket.vn );
	// gps_vd_node->setDoubleValue( gpspacket.vd );
	gps_track_node->setDoubleValue( 90 - atan2(gpspacket.vn, gpspacket.ve)
					* SG_RADIANS_TO_DEGREES );

	if ( console_link_on ) {
	    console_link_gps( &gpspacket );
	}

	if ( log_to_file ) {
	    log_gps( &gpspacket );
	}
    }

    return fresh_data;
}


void GPS_close() {
    switch ( source ) {

    case gpsGPSD:
	// fixme
	break;

    case gpsMNAV:
	// nop
	break;

    case gpsUGFile:
	ugfile_close();
	break;

    default:
	if ( display_on ) {
	    printf("Warning: no gps source defined\n");
	}
    }
}
