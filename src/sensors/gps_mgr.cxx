/**
 * \file: gps_mgr.cxx
 *
 * Front end management interface for reading GPS data.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@flightgear.org
 *
 */


#include "python/pyprops.hxx"

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

#include "comms/display.hxx"
#include "comms/logging.hxx"
#include "comms/remote_link.hxx"
#include "include/globaldefs.h"
#include "init/globals.hxx"
#include "util/coremag.h"
#include "util/myprof.hxx"
#include "util/timing.h"

#include "APM2.hxx"
#include "Goldy2.hxx"
#include "gps_fgfs.hxx"
#include "gps_gpsd.hxx"
#include "gps_mediatek.hxx"
#include "gps_ublox6.hxx"
#include "gps_ublox7.hxx"
#include "ugfile.hxx"

#include "gps_mgr.hxx"

//
// Global variables
//

static double gps_last_time = -31557600.0; // default to t minus one year old

static pyPropertyNode gps_node;
static vector<pyPropertyNode> sections;

static int remote_link_skip = 0;
static int logging_skip = 0;

void GPS_init() {
    gps_node = pyGetNode("/sensors/gps", true);
    
    pyPropertyNode remote_link_node = pyGetNode("/config/remote_link", true);
    pyPropertyNode logging_node = pyGetNode("/config/logging", true);
    remote_link_skip = remote_link_node.getDouble("gps_skip");
    logging_skip = logging_node.getDouble("gps_skip");

    // traverse configured modules
    pyPropertyNode group_node = pyGetNode("/config/sensors/gps_group", true);
    vector<string> children = group_node.getChildren();
    printf("Found %d gps sections\n", (int)children.size());
    for ( unsigned int i = 0; i < children.size(); i++ ) {
	pyPropertyNode section_node = group_node.getChild(children[i].c_str());
	sections.push_back(section_node);
	string source = section_node.getString("source");
	bool enabled = section_node.getBool("enable");
	if ( !enabled ) {
	    continue;
	}
	ostringstream output_path;
	output_path << "/sensors/gps" << '[' << i << ']';
	printf("gps: %d = %s\n", i, source.c_str());
	if ( source == "null" ) {
	    // do nothing
	} else if ( source == "APM2" ) {
	    APM2_gps_init( output_path.str(), &section_node );
	} else if ( source == "fgfs" ) {
	    fgfs_gps_init( output_path.str(), &section_node );
	} else if ( source == "file" ) {
	    ugfile_gps_init( output_path.str(), &section_node );
	} else if ( source == "Goldy2" ) {
	    goldy2_gps_init( output_path.str() );
	} else if ( source == "gpsd" ) {
	    gpsd_init( output_path.str(), &section_node );
	} else if ( source == "mediatek" ) {
	    gps_mediatek3329_init( output_path.str(), &section_node );
	} else if ( source == "ublox6" ) {
	    gps_ublox6_init( output_path.str(), &section_node );
	} else if ( source == "ublox7" ) {
	    gps_ublox7_init( output_path.str(), &section_node );
	} else {
	    printf("Unknown gps source = '%s' in config file\n",
		   source.c_str());
	}
    }
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
    gps_prof.start();

    bool fresh_data = false;
    static int gps_state = 0;

    static int remote_link_count = remote_link_random( remote_link_skip );
    static int logging_count = remote_link_random( logging_skip );

    // traverse configured modules
    for ( unsigned int i = 0; i < sections.size(); i++ ) {
	string source = sections[i].getString("source");
	bool enabled = sections[i].getBool("enable");
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
	} else if ( source == "ublox6" ) {
	    fresh_data = gps_ublox6_update();
	} else if ( source == "ublox7" ) {
	    fresh_data = gps_ublox7_update();
	} else {
	    printf("Unknown gps source = '%s' in config file\n",
		   source.c_str());
	}
	
	if ( fresh_data ) {
	    bool send_remote_link = false;
	    if ( remote_link_on && remote_link_count < 0 ) {
		send_remote_link = true;
		remote_link_count = remote_link_skip;
	    }
	
	    bool send_logging = false;
	    if ( log_to_file && logging_count < 0 ) {
		send_logging = true;
		logging_count = logging_skip;
	    }
	
	    if ( send_remote_link || send_logging ) {
		uint8_t buf[256];
		int size = packer->pack_gps( i, buf );
		if ( send_remote_link ) {
		    remote_link_gps( buf, size );
		}
		if ( send_logging ) {
		    log_gps( buf, size );
		}
	    }
	}
    }

    gps_prof.stop();

    if ( fresh_data ) {
	// for computing gps data age
	gps_last_time = gps_node.getDouble("timestamp");

	if ( remote_link_on ) {
	    remote_link_count--;
	}
	
	if ( log_to_file ) {
	    logging_count--;
	}
    }
    
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
    
    return fresh_data;
}


void GPS_close() {

    // traverse configured modules
    for ( unsigned int i = 0; i < sections.size(); i++ ) {
	string source = sections[i].getString("source");
	bool enabled = sections[i].getBool("enable");
	if ( !enabled ) {
	    continue;
	}
	//printf("i = %d  name = %s source = %s\n",
	//       i, name.c_str(), source.c_str());
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
	} else if ( source == "ublox6" ) {
	    gps_ublox6_close();
	} else if ( source == "ublox7" ) {
	    gps_ublox7_close();
	} else {
	    printf("Unknown gps source = '%s' in config file\n",
		   source.c_str());
	}
    }
}


double GPS_age() {
    return get_Time() - gps_last_time;
}
