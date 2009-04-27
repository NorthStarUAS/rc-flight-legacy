/**
 * \file: gps_mgr.cpp
 *
 * Front end management interface for reading GPS data.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@gmail.com
 *
 * $Id: gps_mgr.cpp,v 1.4 2009/04/27 01:29:09 curt Exp $
 */


#include <math.h>
#include <string.h>

#include "globaldefs.h"

#include "comms/console_link.h"
#include "comms/logging.h"
#include "props/props.hxx"
#include "util/coremag.h"
#include "util/timing.h"

#include "gpsd.h"
#include "mnav.h"
#include "ugfile.h"

#include "gps_mgr.h"

//
// Global variables
//

static struct gps gpspacket;

static gps_source_t source = gpsNone;
static double gps_last_time = -31557600.0; // default to t minus one year old

// gps property nodes
static SGPropertyNode *gps_source_node = NULL;
static SGPropertyNode *gps_time_stamp_node = NULL;
static SGPropertyNode *gps_lat_node = NULL;
static SGPropertyNode *gps_lon_node = NULL;
static SGPropertyNode *gps_alt_node = NULL;
static SGPropertyNode *gps_ve_node = NULL;
static SGPropertyNode *gps_vn_node = NULL;
static SGPropertyNode *gps_vd_node = NULL;
static SGPropertyNode *gps_track_node = NULL;
static SGPropertyNode *gps_unix_sec_node = NULL;
static SGPropertyNode *gps_magvar_deg_node = NULL;

// magnetic variation property nodes
static SGPropertyNode *magvar_init_deg_node = NULL;;

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

    gps_time_stamp_node = fgGetNode("/sensors/gps/time-stamp", true);
    gps_lat_node = fgGetNode("/sensors/gps/latitude-deg", true);
    gps_lon_node = fgGetNode("/sensors/gps/longitude-deg", true);
    gps_alt_node = fgGetNode("/sensors/gps/altitude-m", true);
    gps_ve_node = fgGetNode("/sensors/gps/ve-ms", true);
    gps_vn_node = fgGetNode("/sensors/gps/vn-ms", true);
    gps_vd_node = fgGetNode("/sensors/gps/vd-ms", true);
    gps_track_node = fgGetNode("/sensors/gps/groundtrack-deg", true);
    gps_unix_sec_node = fgGetNode("/sensors/gps/unix-time-sec", true);
    gps_magvar_deg_node = fgGetNode("/sensors/gps/magvar-deg", true);

    // initialize magnetic variation property nodes
    magvar_init_deg_node = fgGetNode("/config/adns/magvar-deg", true);
				
    switch ( source ) {

    case gpsGPSD:
	gpsd_init();
	break;

    case gpsMNAV:
	// mnav conveys little useful date information from the gps,
	// fake it with a recent date that is close enough to compute
	// a reasonable magnetic variation, this should be updated
	// every year or two.
	gpspacket.date = 1240238933; /* Apr 20, 2009 */
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
    static int gps_state = 0;

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

    if ( fresh_data && gps_state == 1 ) {
	gps_last_time = gpspacket.time; // for computing gps data age

	// publish values to property tree
	gps_time_stamp_node->setIntValue( gpspacket.time );
	gps_lat_node->setDoubleValue( gpspacket.lat );
	gps_lon_node->setDoubleValue( gpspacket.lon );
	gps_alt_node->setDoubleValue( gpspacket.alt );
	gps_ve_node->setDoubleValue( gpspacket.ve );
	gps_vn_node->setDoubleValue( gpspacket.vn );
	gps_vd_node->setDoubleValue( gpspacket.vd );
	gps_track_node->setDoubleValue( 90 - atan2(gpspacket.vn, gpspacket.ve)
					* SG_RADIANS_TO_DEGREES );
	gps_unix_sec_node->setDoubleValue( gpspacket.date );

	if ( console_link_on ) {
	    console_link_gps( &gpspacket );
	}

	if ( log_to_file ) {
	    log_gps( &gpspacket );
	}
    } else if ( fresh_data ) {
	const double gps_settle = 10.0;
	static double gps_acq_time = get_Time();
	static double last_time = 0.0;
	double cur_time = get_Time();
	if ( cur_time - gps_acq_time >= gps_settle ) {
	    gps_state = 1;

	    // initialize magnetic variation
	    double magvar_rad = 0.0;
	    if ( strcmp(magvar_init_deg_node->getStringValue(), "auto") == 0
		 || strlen(magvar_init_deg_node->getStringValue()) == 0 )
	    {
		long int jd = unixdate_to_julian_days( gpspacket.date );
		double field[6];
		double gps_lat_rad = gpspacket.lat * SGD_DEGREES_TO_RADIANS;
		double gps_lon_rad = gpspacket.lon * SGD_DEGREES_TO_RADIANS;
		double gps_alt_m   = gpspacket.alt;
		magvar_rad
		    = calc_magvar( gpspacket.lat * SGD_DEGREES_TO_RADIANS,
				   gpspacket.lon * SGD_DEGREES_TO_RADIANS,
				   gpspacket.alt / 1000.0,
				   jd, field );
	    } else {
		magvar_rad = magvar_init_deg_node->getDoubleValue()
		    * SGD_DEGREES_TO_RADIANS;
	    }
	    gps_magvar_deg_node->setDoubleValue( magvar_rad * SG_RADIANS_TO_DEGREES );

	    if ( display_on ) {
		printf("[gps_mgr] gps ready, magvar = %.2f (deg)\n",
		       magvar_rad * SGD_RADIANS_TO_DEGREES );
	    }

	} else {
	    if ( display_on ) {
		if ( cur_time - last_time >= 1.0 ) {
		    printf( "[gps_mgr] gps ready in %.1f seconds.\n",
			    gps_settle - (cur_time - gps_acq_time) );
		    last_time = cur_time;
		}
	    }
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


double GPS_age() {
    return get_Time() - gps_last_time;
}
