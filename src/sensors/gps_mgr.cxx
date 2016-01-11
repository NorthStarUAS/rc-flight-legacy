/**
 * \file: gps_mgr.cxx
 *
 * Front end management interface for reading GPS data.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@flightgear.org
 *
 */


#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include "comms/display.h"
#include "comms/logging.h"
#include "comms/remote_link.h"
#include "include/globaldefs.h"
#include "init/globals.hxx"
#include "python/pyprops.hxx"
#include "util/coremag.h"
#include "util/myprof.h"
#include "util/timing.h"

#include "APM2.hxx"
#include "Goldy2.hxx"
#include "gps_fgfs.hxx"
#include "gps_gpsd.hxx"
#include "gps_mediatek.hxx"
#include "gps_ublox.hxx"
#include "ugfile.hxx"

#include "gps_mgr.hxx"

//
// Global variables
//

static double gps_last_time = -31557600.0; // default to t minus one year old

// gps property nodes
static SGPropertyNode *gps_timestamp_node = NULL;
static SGPropertyNode *gps_unix_sec_node = NULL;
static SGPropertyNode *gps_status_node = NULL;
static SGPropertyNode *gps_magvar_deg_node = NULL;
static SGPropertyNode *gps_settle_node = NULL;

// magnetic variation property nodes
static SGPropertyNode *magvar_init_deg_node = NULL;

// comm property nodes
static SGPropertyNode *gps_console_skip = NULL;
static SGPropertyNode *gps_logging_skip = NULL;

// set system time from gps
static bool set_system_time = false;


void GPS_init() {
    gps_timestamp_node = pyGetNode("/sensors/gps/time-stamp", true);
    gps_unix_sec_node = pyGetNode("/sensors/gps/unix-time-sec", true);
    gps_status_node = pyGetNode("/sensors/gps/status", true);
    gps_magvar_deg_node = pyGetNode("/sensors/gps/magvar-deg", true);
    gps_settle_node = pyGetNode("/sensors/gps/settle", true);
    gps_settle_node->setBoolValue(false);

    // initialize magnetic variation property nodes
    magvar_init_deg_node = pyGetNode("/config/filters/magvar-deg", true);

    // initialize comm nodes
    gps_console_skip = pyGetNode("/config/remote-link/gps-skip", true);
    gps_logging_skip = pyGetNode("/config/logging/gps-skip", true);

    // traverse configured modules
    SGPropertyNode *toplevel = pyGetNode("/config/sensors/gps-group", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "gps" ) {
	    string source = section->getChild("source", 0, true)->getString();
            bool enabled = section->getChild("enable", 0, true)->getBool();
            if ( !enabled ) {
                continue;
            }

	    string basename = "/sensors/";
	    basename += section->getDisplayName();
	    printf("i = %d  name = %s source = %s %s\n",
		   i, name.c_str(), source.c_str(), basename.c_str());
	    if ( source == "null" ) {
		// do nothing
	    } else if ( source == "APM2" ) {
		APM2_gps_init( basename, section );
	    } else if ( source == "fgfs" ) {
		fgfs_gps_init( basename, section );
	    } else if ( source == "file" ) {
		ugfile_gps_init( basename, section );
	    } else if ( source == "Goldy2" ) {
		goldy2_gps_init( basename, section );
	    } else if ( source == "gpsd" ) {
		gpsd_init( basename, section );
	    } else if ( source == "mediatek" ) {
		gps_mediatek3329_init( basename, section );
	    } else if ( source == "ublox" ) {
		gps_ublox_init( basename, section );
	    } else {
		printf("Unknown gps source = '%s' in config file\n",
		       source.c_str());
	    }
	}
    }
}


static void compute_magvar() {
    double magvar_rad = 0.0;
    if ( strcmp(magvar_init_deg_node->getString(), "auto") == 0
	 || strlen(magvar_init_deg_node->getString()) == 0 )
    {
	SGPropertyNode *date_node
	    = pyGetNode("/sensors/gps/unix-time-sec", true);
	SGPropertyNode *lat_node
	    = pyGetNode("/sensors/gps/latitude-deg", true);
	SGPropertyNode *lon_node
	    = pyGetNode("/sensors/gps/longitude-deg", true);
	SGPropertyNode *alt_node
	    = pyGetNode("/sensors/gps/altitude-m", true);
	long int jd = unixdate_to_julian_days( date_node->getIntValue() );
	double field[6];
	magvar_rad
	    = calc_magvar( lat_node->getDouble() * SGD_DEGREES_TO_RADIANS,
			   lon_node->getDouble() * SGD_DEGREES_TO_RADIANS,
			   alt_node->getDouble() / 1000.0,
			   jd, field );
    } else {
	magvar_rad = magvar_init_deg_node->getDouble()
	    * SGD_DEGREES_TO_RADIANS;
    }
    gps_magvar_deg_node->setDoubleValue( magvar_rad * SG_RADIANS_TO_DEGREES );
}


bool GPS_update() {
    gps_prof.start();

    bool fresh_data = false;
    static int gps_state = 0;

    // traverse configured modules
    SGPropertyNode *toplevel = pyGetNode("/config/sensors/gps-group", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "gps" ) {
	    string source = section->getChild("source", 0, true)->getString();
            bool enabled = section->getChild("enable", 0, true)->getBool();
            if ( !enabled ) {
                continue;
            }

	    // printf("i = %d  name = %s source = %s\n",
	    //	   i, name.c_str(), source.c_str());
	    if ( source == "null" ) {
		// do nothing
	    } else if ( source == "APM2" ) {
		fresh_data = APM2_gps_update();
	    } else if ( source == "fgfs" ) {
		fresh_data = fgfs_gps_update();
	    } else if ( source == "file" ) {
		fresh_data = ugfile_get_gps();
	    } else if ( source == "Goldy2" ) {
		fresh_data = goldy2_gps_update();
	    } else if ( source == "gpsd" ) {
		fresh_data = gpsd_get_gps();
	    } else if ( source == "mediatek" ) {
		fresh_data = gps_mediatek3329_update();
	    } else if ( source == "ublox" ) {
		fresh_data = gps_ublox_update();
	    } else {
		printf("Unknown gps source = '%s' in config file\n",
		       source.c_str());
	    }
	}
    }

    gps_prof.stop();

    if ( fresh_data ) {
	// for computing gps data age
	gps_last_time = gps_timestamp_node->getDouble();

	if ( remote_link_on || log_to_file ) {
	    uint8_t buf[256];
	    int size = packetizer->packetize_gps( buf );

	    if ( remote_link_on ) {
		remote_link_gps( buf, size, gps_console_skip->getIntValue() );
	    }

	    if ( log_to_file ) {
		log_gps( buf, size, gps_logging_skip->getIntValue() );
	    }
	}
    }
    if ( gps_status_node->getIntValue() == 2 && !gps_state ) {
	const double gps_settle = 10.0;
	static double gps_acq_time = gps_timestamp_node->getDouble();
	static double last_time = 0.0;
	double cur_time = gps_timestamp_node->getDouble();
	// if ( display_on ) {
	//     printf("gps first aquired = %.3f  cur time = %.3f\n",
	//	   gps_acq_time, cur_time);
	// }

	if ( cur_time - gps_acq_time >= gps_settle ) {
	    gps_state = 1;
	    gps_settle_node->setBoolValue(true);

	    // initialize magnetic variation
	    compute_magvar();

	    // set the host system clock if we have a unix-time-sec
	    // value and if that seems substantially newer than the
	    // host clock
	    struct timeval system_time;
	    gettimeofday( &system_time, NULL );
	    double system_clock = (double)system_time.tv_sec +
		(double)system_time.tv_usec / 1000000;
	    double gps_clock = gps_unix_sec_node->getDouble();
	    if ( fabs( system_clock - gps_clock ) > 300 ) {
		// if system clock is off from gps clock by more than
		// 300 seconds (5 minutes) attempt to set system clock
		// from gps clock
		struct timeval newtime;
		newtime.tv_sec = gps_clock;
		newtime.tv_usec = (gps_clock - (double)newtime.tv_sec)
		    * 1000000;
		if ( display_on ) {
		    printf("System clock: %.2f\n", system_clock);
		    printf("GPS clock: %.2f\n", gps_clock);
		    printf("Setting system clock to sec: %d usec: %d\n",
			   newtime.tv_sec, newtime.tv_usec);
		}
		if ( settimeofday( &newtime, NULL ) != 0 ) {
		    // failed to change system clock
		    perror("Set time");
		}
	    }

	    
	    if ( display_on ) {
		printf("[gps_mgr] gps ready, magvar = %.2f (deg)\n",
		       gps_magvar_deg_node->getDouble() );
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
    SGPropertyNode *toplevel = pyGetNode("/config/sensors/gps-group", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "gps" ) {
	    string source = section->getChild("source", 0, true)->getString();
            bool enabled = section->getChild("enable", 0, true)->getBool();
            if ( !enabled ) {
                continue;
            }

	    printf("i = %d  name = %s source = %s\n",
		   i, name.c_str(), source.c_str());
	    if ( source == "null" ) {
		// do nothing
	    } else if ( source == "APM2" ) {
		APM2_gps_close();
	    } else if ( source == "fgfs" ) {
		fgfs_gps_close();
	    } else if ( source == "file" ) {
		ugfile_close();
	    } else if ( source == "Goldy2" ) {
		goldy2_gps_close();
	    } else if ( source == "gpsd" ) {
		// fixme
	    } else if ( source == "mediatek" ) {
		gps_mediatek3329_close();
	    } else if ( source == "ublox" ) {
		gps_ublox_close();
	    } else {
		printf("Unknown gps source = '%s' in config file\n",
		       source.c_str());
	    }
	}
    }
}


double GPS_age() {
    return get_Time() - gps_last_time;
}
