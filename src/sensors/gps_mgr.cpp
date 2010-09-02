/**
 * \file: gps_mgr.cpp
 *
 * Front end management interface for reading GPS data.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@gmail.com
 *
 * $Id: gps_mgr.cpp,v 1.7 2009/08/25 15:04:01 curt Exp $
 */


#include <math.h>
#include <stdio.h>
#include <string.h>

#include "include/ugear_config.h"

#include "comms/logging.h"
#include "comms/remote_link.h"
#include "include/globaldefs.h"
#include "main/globals.hxx"
#include "props/props.hxx"
#include "util/coremag.h"
#include "util/myprof.h"
#include "util/timing.h"

#include "gps_fgfs.hxx"
#include "gps_gpsd.h"
#ifdef ENABLE_MNAV_SENSOR
#  include "mnav.h"
#endif // ENABLE_MNAV_SENSOR
#include "gps_ublox5.h"
#include "ugfile.h"

#include "gps_mgr.h"

//
// Global variables
//

static double gps_last_time = -31557600.0; // default to t minus one year old

// gps property nodes
static SGPropertyNode *gps_timestamp_node = NULL;
static SGPropertyNode *gps_track_node = NULL;
static SGPropertyNode *gps_magvar_deg_node = NULL;

// magnetic variation property nodes
static SGPropertyNode *magvar_init_deg_node = NULL;

// comm property nodes
static SGPropertyNode *gps_console_skip = NULL;
static SGPropertyNode *gps_logging_skip = NULL;


void GPS_init() {
    gps_timestamp_node = fgGetNode("/sensors/gps/time-stamp", true);
    gps_track_node = fgGetNode("/sensors/gps/groundtrack-deg", true);
    gps_magvar_deg_node = fgGetNode("/sensors/gps/magvar-deg", true);

    // initialize magnetic variation property nodes
    magvar_init_deg_node = fgGetNode("/config/filters/magvar-deg", true);

    // initialize comm nodes
    gps_console_skip = fgGetNode("/config/console/gps-skip", true);
    gps_logging_skip = fgGetNode("/config/logging/gps-skip", true);

    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/sensors/gps-group", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "gps" ) {
	    string source = section->getChild("source", 0, true)->getStringValue();
            bool enabled = section->getChild("enable", 0, true)->getBoolValue();
            if ( !enabled ) {
                continue;
            }

	    string basename = "/sensors/";
	    basename += section->getDisplayName();
	    printf("i = %d  name = %s source = %s %s\n",
		   i, name.c_str(), source.c_str(), basename.c_str());
	    if ( source == "null" ) {
		// do nothing
	    } else if ( source == "fgfs" ) {
		fgfs_gps_init( basename, section );
	    } else if ( source == "file" ) {
		ugfile_gps_init( basename, section );
	    } else if ( source == "gpsd" ) {
		gpsd_init( basename, section );
#ifdef ENABLE_MNAV_SENSOR
	    } else if ( source == "mnav" ) {
		mnav_gps_init( basename );
#endif // ENABLE_MNAV_SENSOR
	    } else if ( source == "ublox5" ) {
		gps_ublox5_init( basename, section );
	    } else {
		printf("Unknown imu source = '%s' in config file\n",
		       source.c_str());
	    }
	}
    }
}


static void compute_magvar() {
    double magvar_rad = 0.0;
    if ( strcmp(magvar_init_deg_node->getStringValue(), "auto") == 0
	 || strlen(magvar_init_deg_node->getStringValue()) == 0 )
    {
	SGPropertyNode *date_node
	    = fgGetNode("/sensors/gps/unix-time-sec", true);
	SGPropertyNode *lat_node
	    = fgGetNode("/sensors/gps/latitude-deg", true);
	SGPropertyNode *lon_node
	    = fgGetNode("/sensors/gps/longitude-deg", true);
	SGPropertyNode *alt_node
	    = fgGetNode("/sensors/gps/altitude-m", true);
	long int jd = unixdate_to_julian_days( date_node->getIntValue() );
	double field[6];
	magvar_rad
	    = calc_magvar( lat_node->getDoubleValue() * SGD_DEGREES_TO_RADIANS,
			   lon_node->getDoubleValue() * SGD_DEGREES_TO_RADIANS,
			   alt_node->getDoubleValue() / 1000.0,
			   jd, field );
    } else {
	magvar_rad = magvar_init_deg_node->getDoubleValue()
	    * SGD_DEGREES_TO_RADIANS;
    }
    gps_magvar_deg_node->setDoubleValue( magvar_rad * SG_RADIANS_TO_DEGREES );
}


bool GPS_update() {
    gps_prof.start();

    bool fresh_data = false;
    static int gps_state = 0;

    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/sensors/gps-group", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "gps" ) {
	    string source = section->getChild("source", 0, true)->getStringValue();
            bool enabled = section->getChild("enable", 0, true)->getBoolValue();
            if ( !enabled ) {
                continue;
            }

	    // printf("i = %d  name = %s source = %s\n",
	    //	   i, name.c_str(), source.c_str());
	    if ( source == "null" ) {
		// do nothing
	    } else if ( source == "fgfs" ) {
		fresh_data = fgfs_gps_update();
	    } else if ( source == "file" ) {
		fresh_data = ugfile_get_gps();
	    } else if ( source == "gpsd" ) {
		fresh_data = gpsd_get_gps();
#ifdef ENABLE_MNAV_SENSOR
	    } else if ( source == "mnav" ) {
		fresh_data = mnav_get_gps();
#endif // ENABLE_MNAV_SENSOR
	    } else if ( source == "ublox5" ) {
		fresh_data = gps_ublox5_update();
	    } else {
		printf("Unknown imu source = '%s' in config file\n",
		       source.c_str());
	    }
	}
    }

    gps_prof.stop();

    if ( fresh_data && gps_state == 1 ) {
	// for computing gps data age
	gps_last_time = gps_timestamp_node->getDoubleValue();

	if ( console_link_on || log_to_file ) {
	    uint8_t buf[256];
	    int size = packetizer->packetize_gps( buf );

	    if ( console_link_on ) {
		console_link_gps( buf, size, gps_console_skip->getIntValue() );
	    }

	    if ( log_to_file ) {
		log_gps( buf, size, gps_logging_skip->getIntValue() );
	    }
	}
    } else if ( fresh_data ) {
	const double gps_settle = 10.0;
	static double gps_acq_time = gps_timestamp_node->getDoubleValue();
	static double last_time = 0.0;
	double cur_time = gps_timestamp_node->getDoubleValue();
	// if ( display_on ) {
	//     printf("gps first aquired = %.3f  cur time = %.3f\n",
	//	   gps_acq_time, cur_time);
	// }

	if ( cur_time - gps_acq_time >= gps_settle ) {
	    gps_state = 1;

	    // initialize magnetic variation
	    compute_magvar();

	    if ( display_on ) {
		printf("[gps_mgr] gps ready, magvar = %.2f (deg)\n",
		       gps_magvar_deg_node->getDoubleValue() );
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

    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/sensors/gps-group", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "gps" ) {
	    string source = section->getChild("source", 0, true)->getStringValue();
            bool enabled = section->getChild("enable", 0, true)->getBoolValue();
            if ( !enabled ) {
                continue;
            }

	    printf("i = %d  name = %s source = %s\n",
		   i, name.c_str(), source.c_str());
	    if ( source == "null" ) {
		// do nothing
	    } else if ( source == "fgfs" ) {
		fgfs_gps_close();
	    } else if ( source == "file" ) {
		ugfile_close();
	    } else if ( source == "gpsd" ) {
		// fixme
#ifdef ENABLE_MNAV_SENSOR
	    } else if ( source == "mnav" ) {
		// nop
#endif // ENABLE_MNAV_SENSOR
	    } else if ( source == "ublox5" ) {
		gps_ublox5_close();
	    } else {
		printf("Unknown imu source = '%s' in config file\n",
		       source.c_str());
	    }
	}
    }
}


double GPS_age() {
    return get_Time() - gps_last_time;
}
