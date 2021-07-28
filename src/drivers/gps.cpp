 /**
 * \file: gps_mgr.cpp
 *
 * Front end management interface for reading GPS data.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@flightgear.org
 *
 */


#include <pybind11/pybind11.h>
namespace py = pybind11;

#include <props2.h>

#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include <string>
using std::string;

#include "include/globaldefs.h"
#include "filters/nav_common/coremag.h"
#include "util/timing.h"

#include "gps.h"

void gps_helper_t::init(DocPointerWrapper d) {
    PropertyNode("/").set_Document(d);

    gps_node = PropertyNode( "/sensors/gps" );
    // init master gps timestamp to one year ago
    gps_node.setDouble("timestamp", -31557600.0);
}

void gps_helper_t::compute_magvar() {
    double magvar_rad = 0.0;

    PropertyNode config_node( "/config" );
    
    if ( ! config_node.hasChild("magvar_deg") ||
	 config_node.getString("magvar_deg") == "auto" )
    {
	long int jd = unixdate_to_julian_days( gps_node.getDouble("unix_time_sec") );
	double field[6];
	magvar_rad
	    = calc_magvar( gps_node.getDouble("latitude_deg")
			   * SGD_DEGREES_TO_RADIANS,
			   gps_node.getDouble("longitude_deg")
			   * SGD_DEGREES_TO_RADIANS,
			   gps_node.getFloat("altitude_m") / 1000.0,
			   jd, field );
    } else {
	magvar_rad = config_node.getFloat("magvar_deg")
	    * SGD_DEGREES_TO_RADIANS;
    }
    gps_node.setFloat( "magvar_deg", magvar_rad * SG_RADIANS_TO_DEGREES );
}

double gps_helper_t::gps_age() {
    return get_Time() - gps_node.getDouble("timestamp");
}

void gps_helper_t::update(bool verbose) {
    // FIXME: should this be "fixType" == 3?
    if ( gps_node.getInt("status") == 2 && !gps_state ) {
	const double gps_settle = 10.0;
	double cur_time = gps_node.getDouble("timestamp");
        if ( gps_acq_time < 0.01 ) {
            gps_acq_time = cur_time;
        }
	// if ( verbose ) {
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
		if ( verbose ) {
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

	    if ( verbose ) {
		printf("[gps_mgr] gps ready, magvar = %.2f (deg)\n",
		       gps_node.getFloat("magvar_deg") );
	    }
	} else {
	    if ( verbose ) {
		if ( cur_time - last_time >= 1.0 ) {
		    printf( "[gps_mgr] gps ready in %.1f seconds.\n",
			    gps_settle - (cur_time - gps_acq_time) );
		    last_time = cur_time;
		}
	    }
	}
    }

    gps_node.setFloat("data_age", gps_age());
}

PYBIND11_MODULE(gps_helper, m) {
    py::class_<gps_helper_t>(m, "gps_helper")
        .def(py::init<>())
        .def("init", &gps_helper_t::init)
        .def("update", &gps_helper_t::update) 
        .def("gps_age", &gps_helper_t::gps_age)
   ;
}
