//
// ugear_timer.cxx - top level "main" program
//
// Written by Curtis Olson, curtolson <at> gmail <dot> com.
// Started 2007.
// This code is released into the public domain.
// 

#include <stdio.h>
#include <sys/types.h>

#include <sys/stat.h>
#include <fcntl.h>
#include <sys/time.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <sys/resource.h>
#include <unistd.h>

#include <string>

#include <umngnss/adns.h>

#include "include/ugear_config.h"

#include "actuators/act_mgr.h"
#include "comms/console_link.h"
#include "comms/logging.h"
#include "comms/netSocket.h"	// netInit()
#include "control/control.h"
#include "control/route_mgr.hxx"
#include "filters/filter_mgr.h"
#include "health/health.h"
#include "include/globaldefs.h"
#include "props/props.hxx"
#include "props/props_io.hxx"
#include "sensors/airdata_mgr.h"
#include "sensors/gps_mgr.h"
#include "sensors/mnav.h"
#include "util/exception.hxx"
#include "util/myprof.h"
#include "util/sg_path.hxx"
#include "util/timing.h"

#include "globals.hxx"
#include "imu_mgr.hxx"

using std::string;


//
// Configuration settings
//

static const int HEARTBEAT_HZ = 50;	 // master clock rate

static bool log_servo_out  = true;    // log outgoing servo commands by default
static bool enable_control = false;   // autopilot control module enabled/disabled
static bool enable_route   = false;   // route module enabled/disabled
static bool enable_telnet  = false;   // telnet command/monitor interface
static bool initial_home   = false;   // initial home position determined
static double gps_timeout_sec = 9.0;  // nav algorithm gps timeout
static double lost_link_sec = 59.0;   // lost link timeout

//
// usage message
//
void usage()
{
    printf("\n./ugear --option1 on/off --option2 on/off --option3 ... \n");
    printf("--log-dir path       : enable onboard data logging to path\n");
    printf("--log-servo in/out   : specify which servo data to log (out=default)\n");
    printf("--mnav <device>      : specify mnav communication device\n");
    printf("--console <dev>      : specify console device and enable link\n");
    printf("--display on/off     : dump periodic data to display\n");	
    printf("--help               : display this help messages\n\n");
    
    _exit(0);	
}	


// Main work routine.  Please note that by default, the timer signal
// is ignored while the signal handler routine is being executed.
// This behavior can be changed by setting the SA_NODEFER flag in the
// sa structure.
void timer_handler (int signum)
{
    main_prof.start();

    // master "dt"
    static double last_time = 0.0;
    double current_time = get_Time();
    // double dt = current_time - last_time;
    last_time = current_time;

    static int health_counter = 0;
    static int display_counter = 0;
    static int ap_counter = 0;
    static int route_counter = 0;
    static int command_counter = 0;
    static int flush_counter = 0;
    static double last_command_time = 0.0;

    static int count = 0;

    count++;
    // printf ("timer expired %d times\n", count);

    health_counter++;
    display_counter++;
    ap_counter++;
    route_counter++;
    command_counter++;
    flush_counter++;

    //
    // Sensor input section
    //

    // Fetch the next data packet from the IMU.
    bool fresh_imu_data = IMU_update();

    // Fetch air data if available
    AirData_update();

    // Fetch GPS data if available.
    GPS_update();

#if 0 // FIXME: this needs to be handled in a more generic way
    mnav_manual_override_check();
#endif

    //
    // Attitude Determination and Navigation section
    //

    Filter_update( fresh_imu_data );

    // check gps data age.  The nav filter continues to run, but the
    // results are marked as NotValid if the most recent gps data
    // becomes too old.
    if ( GPS_age() > gps_timeout_sec ) {
	SGPropertyNode *filter_status_node
	    = fgGetNode("/status/navigation", true);
	filter_status_node->setStringValue("invalid");
    }

    // initial home is most recent gps result after being alive with a
    // solution for 20 seconds
    if ( !initial_home && GPS_age() < 2.0 ) {
	SGPropertyNode *gps_lat_node
	    = fgGetNode("/sensors/gps/latitude-deg", true);
	SGPropertyNode *gps_lon_node
	    = fgGetNode("/sensors/gps/longitude-deg", true);
	SGWayPoint wp( gps_lon_node->getDoubleValue(),
		       gps_lat_node->getDoubleValue(),
		       -9999.9 );
	if ( route_mgr.update_home(wp, 0.0, true /*force update*/) ) {
	    initial_home = true;
	}
    }

    /* FIXME: TEMPORARY */ logging_navstate();

    //
    // Read commands from ground station section
    //

    if ( console_link_on ) {
	bool read_command = false;

	// check for incoming command data (5hz)
	if ( command_counter >= (HEARTBEAT_HZ / 5) ) {
	    command_counter = 0;
	    if ( console_link_command() ) {
		read_command = true;
		last_command_time = current_time;
		// FIXME: we shouldn't necessarily assume route
		// mode just because we read a command from the
		// ground station
		route_mgr.set_route_mode();
	    }
	}
	if ( read_command
	     && current_time > last_command_time + lost_link_sec
	     && route_mgr.get_route_mode() != FGRouteMgr::GoHome )
        {
	    // We have previously established a positive link with the
	    // groundstation, but it's been lost_link seconds since
	    // the last command received and we aren't already in
	    // GoHome mode.  Console link is assumed to be down or
	    // we've flown out of radio modem range.  Switch to fly
	    // home mode.  Ground station operator will need to send a
	    // resume route command to resume the route.
	    route_mgr.set_home_mode();
	}
    }

    //
    // Read commands from telnet interface
    //

    if ( enable_telnet ) {
	telnet->process();
    }

    //
    // Control section
    //

    if ( enable_route ) {
	// route updates at 5 hz
	if ( route_counter >= (HEARTBEAT_HZ / 5) ) {
	    route_counter = 0;
	    route_mgr_prof.start();
	    route_mgr.update();
	    route_mgr_prof.stop();
	}
    }

    if ( enable_control ) {
	// autopilot update at 25 hz
	if ( ap_counter >= (HEARTBEAT_HZ / 25) ) { 
	    ap_counter = 0;
	    control_prof.start();
	    control_update(0);
	    control_prof.stop();
	}

	Actuator_update();
    }

    //
    // Data logging and Telemetry dump section
    //

    // health status (update at 1hz)
    if ( health_counter >= (HEARTBEAT_HZ / 1) ) {
	health_prof.start();
	health_counter = 0;
	health_update();
	health_prof.stop();
    }

    // sensor summary dispay (update at 0.5hz)
    if ( display_on && display_counter
	 >= (HEARTBEAT_HZ * 2 /* divide by 0.5 */) )
    {
	display_counter = 0;
	display_message();
	imu_prof.stats( "IMU " );
	gps_prof.stats( "GPS " );
	air_prof.stats( "AIR " );
	filter_prof.stats( "FILT" );
	if ( enable_control ) {
	    control_prof.stats( "CNTL" );
	}
	health_prof.stats( "HLTH" );
	main_prof.stats( "MAIN" );
    }

    // round robin flushing of logging streams (update at 0.5hz)
    if ( flush_counter >= (HEARTBEAT_HZ * 2 /* divide by 0.5 */) ) {
	flush_counter = 0;
	static int flush_state = 0;
	if ( log_to_file ) {
	    switch ( flush_state ) {
	    case 0:
		flush_gps();
		flush_state++;
		break;
	    case 1:
		flush_imu();
		flush_state++;
		break;
	    case 2:
		flush_airdata();
		flush_state++;
		break;
	    case 3:
		flush_filter();
		flush_state++;
		break;
	    case 4:
		flush_actuator();
		flush_state++;
		break;
	    case 5:
		flush_pilot();
		flush_state++;
		break;
	    case 6:
		flush_ap();
		flush_state++;
		break;
	    default:
		flush_state = 0;
	    }
	}
    }

    main_prof.stop();
}


//
// main ...
//
int main( int argc, char **argv )
{
    int iarg;

    // structures for setting up timer handler
    struct sigaction sa;
    struct itimerval timer;

    // initialize network library
    netInit( NULL, NULL );

    // initialize properties
    props = new SGPropertyNode;

    string root = ".";
    SGPropertyNode *root_node = fgGetNode("/config/root-path", true);
    root_node->setStringValue( root.c_str() );

    UGGlobals_init();

    // load master config file
    SGPath master( root );
    master.append( "config.xml" );
    try {
        readProperties( master.c_str(), props);
        printf("Loaded configuration from %s\n", master.c_str());
    } catch (const sg_exception &exc) {
        printf("\n");
        printf("*** Cannot load master config file: %s\n", master.c_str());
        printf("\n");
        sleep(1);
    }

    // extract configuration values from the property tree (which is
    // now populated with the master config.xml data.  Do this before
    // the command line processing so that any options specified on
    // the command line will override what is in the config.xml file.

    SGPropertyNode *p;

    p = fgGetNode("/config/logging/path", true);
    log_path.set( p->getStringValue() );
    p = fgGetNode("/config/logging/enable", true);
    log_to_file = p->getBoolValue();
    printf("log path = %s enabled = %d\n", log_path.c_str(), log_to_file);

    p = fgGetNode("/config/telnet/enable", true);
    enable_telnet = p->getBoolValue();
    if ( enable_telnet ) {
	p = fgGetNode("/config/telnet/port", true);
	if ( p->getIntValue() > 0 ) {
	    telnet = new UGTelnet( p->getIntValue() );
	    telnet->open();
	} else {
	    printf("No telnet port defined, disabling telnet interface\n");
	    enable_telnet = false;
	}
    }

    p = fgGetNode("/config/filters/gps-timeout-sec");
    if ( p != NULL && p->getDoubleValue() > 0.0001 ) {
	// stick with the default if nothing valid specified
	gps_timeout_sec = p->getDoubleValue();
    }
    printf("gps timeout = %.1f\n", gps_timeout_sec);

    p = fgGetNode("/config/console/lost-link-timeout-sec");
    if ( p != NULL && p->getDoubleValue() > 0.0001 ) {
	// stick with the default if nothing valid specified
	lost_link_sec = p->getDoubleValue();
    }
    printf("lost link timeout = %.1f\n", lost_link_sec);
    
    p = fgGetNode("/config/autopilot/enable", true);
    enable_control = p->getBoolValue();

    p = fgGetNode("/config/route/enable", true);
    enable_route = p->getBoolValue();

    // Parse the command line
    for ( iarg = 1; iarg < argc; iarg++ ) {
        if ( !strcmp(argv[iarg], "--log-dir" )  ) {
            ++iarg;
            log_path.set( argv[iarg] );
            log_to_file = true;
        } else if ( !strcmp(argv[iarg],"--log-servo") ) {
            ++iarg;
            if ( !strcmp(argv[iarg], "out") ) log_servo_out = true;
            if ( !strcmp(argv[iarg], "in") ) log_servo_out = false;
        } else if ( !strcmp(argv[iarg], "--mnav" )  ) {
            ++iarg;
	    p = fgGetNode("/config/sensors/mnav/device", true);
	    p->setStringValue( argv[iarg] );
        } else if ( !strcmp(argv[iarg], "--console" )  ) {
            ++iarg;
	    console_link_on = true;
	    p = fgGetNode("/config/console/device", true);
	    p->setStringValue( argv[iarg] );
        } else if ( !strcmp(argv[iarg],"--display") ) {
            ++iarg;
            if ( !strcmp(argv[iarg], "on") ) display_on = true;
            if ( !strcmp(argv[iarg], "off") ) display_on = false;
        } else if ( !strcmp(argv[iarg], "--help") ) {
            usage();
        } else {
            printf("Unknown option \"%s\"\n", argv[iarg]);
            usage();
        }
    }

    // open console link if requested
    if ( console_link_on ) {
        console_link_init();
    }

    // open logging files if requested
    if ( log_to_file ) {
        if ( !logging_init() ) {
            printf("Warning: error opening one or more data files, logging disabled\n");
            log_to_file = false;
        }
    }

    /* FIXME: TEMPORARY */ logging_navstate_init();

    // Initialize communication with the selected IMU
    IMU_init();

    // Initialize communication with the selected air data sensor
    AirData_init();

    // Initialize communication with the selected GPS
    GPS_init();

    // Initialize any defined filter modules
    Filter_init();

    // init system health and status monitor
    health_init();

    if ( enable_control ) {
        // initialize the autopilot
        control_init();

	// initialize the actuators
	Actuator_init();
    }

    if ( enable_route ) {
        // initialize the route manager
        route_mgr.init();
    }

#ifndef BATCH_MODE
    // Install timer_handler as the signal handler for SIGALRM (alarm
    // timing is based on wall clock)
    memset (&sa, 0, sizeof (sa));
    sa.sa_handler = &timer_handler;
    sigaction (SIGALRM, &sa, NULL);

    // Configure the timer to expire after 10,000 usec (1/100th of a second)
    timer.it_value.tv_sec = 0;
    timer.it_value.tv_usec = (1000000 / HEARTBEAT_HZ);
    // ... and every 10 msec after that (100hz)
    timer.it_interval.tv_sec = 0;
    timer.it_interval.tv_usec = (1000000 / HEARTBEAT_HZ);
    // Start a real timer. It counts down based on the wall clock
    setitimer (ITIMER_REAL, &timer, NULL);
#endif

    printf("Everything inited ... ready to run\n");

    // enter a do nothing "main" loop.  The real work is done in the
    // timer_handler() callback which is run every time the alarm is
    // generated (100hz default)
    while ( true ) {
#ifndef BATCH_MODE
	// printf("main(): sleeping\n");
	sleep(1);
#else
	timer_handler( 0 );
#endif
    }

    // close and exit
    Filter_close();
    IMU_close();
    GPS_close();
    AirData_close();
    if ( enable_control ) {
	control_close();
	Actuator_close();
    }
}


