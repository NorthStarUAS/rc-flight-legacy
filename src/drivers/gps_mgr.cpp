 /**
 * \file: gps_mgr.cpp
 *
 * Front end management interface for reading GPS data.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@flightgear.org
 *
 */


#include <pyprops.h>

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include <sstream>
#include <string>
#include <vector>
using std::ostringstream;
using std::string;
using std::vector;

#include "comms/aura_messages.h"
#include "comms/display.h"
#include "include/globaldefs.h"
#include "init/globals.h"
#include "util/coremag.h"
#include "util/myprof.h"
#include "util/timing.h"

#include "gps_ublox6.h"
#include "gps_ublox8.h"

#include "gps_mgr.h"

//
// Global variables
//

static pyPropertyNode gps_node;
static vector<pyPropertyNode> sections;
static vector<pyPropertyNode> outputs;


void GPS_init() {
    gps_node = pyGetNode("/sensors/gps", true);
    // init master gps timestamp to one year ago
    gps_node.setDouble("timestamp", -31557600.0);
}


static void compute_magvar() {
    double magvar_rad = 0.0;

    pyPropertyNode config_node = pyGetNode("/config", true);
    
    if ( ! config_node.hasChild("magvar_deg") ||
	 config_node.getString("magvar_deg") == "auto" )
    {
	long int jd = unixdate_to_julian_days( gps_node.getLong("unix_time_sec") );
	double field[6];
	magvar_rad
	    = calc_magvar( gps_node.getDouble("latitude_deg")
			   * SGD_DEGREES_TO_RADIANS,
			   gps_node.getDouble("longitude_deg")
			   * SGD_DEGREES_TO_RADIANS,
			   gps_node.getDouble("altitude_m") / 1000.0,
			   jd, field );
    } else {
	magvar_rad = config_node.getDouble("magvar_deg")
	    * SGD_DEGREES_TO_RADIANS;
    }
    gps_node.setDouble( "magvar_deg", magvar_rad * SG_RADIANS_TO_DEGREES );
}


bool GPS_update() {
    static int gps_state = 0;

    // FIXME: should this be "fixType" == 3?
    if ( gps_node.getLong("status") == 2 && !gps_state ) {
	const double gps_settle = 10.0;
	static double gps_acq_time = gps_node.getDouble("timestamp");
	static double last_time = 0.0;
	double cur_time = gps_node.getDouble("timestamp");
	// if ( display_on ) {
	//     printf("gps first aquired = %.3f  cur time = %.3f\n",
	//	   gps_acq_time, cur_time);
	// }

	if ( cur_time - gps_acq_time >= gps_settle ) {
	    gps_state = 1;
	    gps_node.setBool("settle", true);

	    // initialize magnetic variation
	    compute_magvar();

	    // set the host system clock if we have a unix-time-sec
	    // value and if that seems substantially newer than the
	    // host clock
	    struct timeval system_time;
	    gettimeofday( &system_time, NULL );
	    double system_clock = (double)system_time.tv_sec +
		(double)system_time.tv_usec / 1000000;
	    double gps_clock = gps_node.getDouble("unix_time_sec");
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
		    printf("Setting system clock to sec: %ld usec: %ld\n",
			   newtime.tv_sec, newtime.tv_usec);
		}
		if ( settimeofday( &newtime, NULL ) != 0 ) {
		    // failed to change system clock
		    perror("Set time");
		}
	    }

	    if ( display_on ) {
		printf("[gps_mgr] gps ready, magvar = %.2f (deg)\n",
		       gps_node.getDouble("magvar_deg") );
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

    gps_node.setDouble("data_age", GPS_age());
    
    return true;
}

double GPS_age() {
    return get_Time() - gps_node.getDouble("timestamp");
}
