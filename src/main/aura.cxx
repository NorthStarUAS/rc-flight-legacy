//
// aura.cxx - top level "main" program
//
// Written by Curtis Olson, curtolson <at> gmail <dot> com.
// Started 2007.
// This code is released into the public domain.
// 

#include "python/python_sys.hxx"
#include "python/pyprops.hxx"

#include <stdio.h>
#include <sys/types.h>

#include <sys/stat.h>
#include <fcntl.h>
#include <sys/time.h>
#include <stdlib.h>
#include <string.h>
#include <sys/resource.h>
#include <unistd.h>

#include <string>
using std::string;

#include "include/aura_config.h"

#include "actuators/act_mgr.hxx"
#include "comms/display.hxx"
#include "comms/logging.hxx"
#include "comms/remote_link.hxx"
#include "comms/netSocket.h"	// netInit()
#include "control/cas.hxx"
#include "control/control.hxx"
#include "control/route_mgr.hxx"
#include "filters/filter_mgr.hxx"
#include "health/health.hxx"
#include "include/globaldefs.h"
#include "init/globals.hxx"
#include "payload/payload_mgr.hxx"
#include "sensors/airdata_mgr.hxx"
#include "sensors/imu_mgr.hxx"
#include "sensors/gps_mgr.hxx"
#include "sensors/pilot_mgr.hxx"
#include "util/exception.hxx"
#include "util/myprof.hxx"
#include "util/sg_path.hxx"
#include "util/timing.h"

#include "sensors/APM2.hxx"


//
// Configuration settings
//

static const int HEARTBEAT_HZ = 100;	 // master clock rate

static bool enable_mission = true;    // mission mgr module enabled/disabled
static bool enable_cas     = false;   // cas module enabled/disabled
static bool enable_telnet  = false;   // telnet command/monitor interface
static bool enable_pointing = false;  // pan/tilt pointing module
static double gps_timeout_sec = 9.0;  // nav algorithm gps timeout

// property nodes
static pyPropertyNode status_node;

// debug main loop "block" on gumstix verdex
myprofile debug1;
myprofile debug2;
myprofile debug2a;
myprofile debug2b;
myprofile debug2c;
myprofile debug2d;
myprofile debug3;
myprofile debug4;
myprofile debug5;
myprofile debug6;
myprofile debug7;

//
// usage message
//
void usage(char *progname)
{
    printf("\n%s --option1 on/off --option2 on/off --option3 ... \n", progname);
    printf("--config path        : path to location of configuration file tree\n");
    printf("--log-dir path       : enable onboard data logging to path\n");
    printf("--log-servo in/out   : specify which servo data to log (out=default)\n");
    printf("--remote-link on/off : remote link enable or disabled\n");
    printf("--display on/off     : dump periodic data to display\n");	
    printf("--help               : display this help messages\n\n");
    
    _exit(0);	
}	


void main_work_loop()
{
    debug1.start();

    // read the APM2 sensor head until we receive an IMU packet
    sync_prof.start();
    APM2_update();
    sync_prof.stop();
    
    main_prof.start();
    // master "dt"
    static double last_time = 0.0;
    double current_time = get_Time();
    double dt = current_time - last_time;
    last_time = current_time;

    status_node.setDouble("frame_time", current_time);
    
    static int health_counter = 0;
    static int display_counter = 0;
    static int flush_counter = 0;

    static int count = 0;

    count++;
    // printf ("timer expired %d times\n", count);

    health_counter++;
    display_counter++;
    flush_counter++;

    debug1.stop();

    debug2.start();

    //
    // Sensor input section
    //

    debug2a.start();
    // Fetch the next data packet from the IMU.
    bool fresh_imu_data = IMU_update();
    debug2a.stop();

    debug2b.start();
    // Fetch air data if available
    AirData_update();
    debug2b.stop();

    debug2c.start();
    // Fetch GPS data if available.
    GPS_update();
    debug2c.stop();

    debug2d.start();
    // Fetch Pilot Inputs
    PilotInput_update();
    debug2d.stop();

    debug2.stop();

    debug3.start();

    //
    // Attitude Determination and Navigation section
    //
    if ( fresh_imu_data ) {
	Filter_update();
    }

    // check gps data age.  The nav filter continues to run, but the
    // results are marked as NotValid if the most recent gps data
    // becomes too old.
    if ( GPS_age() > gps_timeout_sec ) {
	status_node.setString("navigation", "invalid");
    }

    /* FIXME: TEMPORARY */ /* logging_navstate(); */

    debug3.stop();

    debug4.start();

    //
    // Read commands from ground station section
    //

    if ( remote_link_on ) {
	// check for incoming command data
	remote_link_command();

	// dribble a bit more out of the serial port if there is
	// something pending
	remote_link_flush_serial();
    }

    debug4.stop();

    debug5.start();

    //
    // Read commands from telnet interface
    //

    if ( enable_telnet ) {
	telnet->process();
    }

    debug5.stop();

    // if ( enable_pointing ) {
    // 	// Update pointing module
    // 	ati_pointing_update( dt );
    // }

    debug6.start();

    //
    // Control section
    //

    if ( enable_mission ) {
	mission_mgr->update();
    }

    if ( enable_cas ) {
	cas.update();
    }

    control_prof.start();
    control_update(dt);
    control_prof.stop();
    Actuator_update();

    debug6.stop();

    debug7.start();

    //
    // Data logging and Telemetry dump section
    //

    // health status (update at 10hz)
    if ( health_counter >= (HEARTBEAT_HZ / 10) ) {
	health_prof.start();
	health_counter = 0;
	health_update();
	health_prof.stop();
    }

    payload_mgr.update();

    // sensor summary dispay (update at 0.5hz)
    if ( display_on && display_counter
	 >= (HEARTBEAT_HZ * 2 /* divide by 0.5 */) )
    {
	display_counter = 0;
	display_message();
	imu_prof.stats();
	gps_prof.stats();
	air_prof.stats();
	filter_prof.stats();
	control_prof.stats();
	health_prof.stats();
	datalog_prof.stats();
	sync_prof.stats();
	main_prof.stats();
    }

    // flush of logging stream (update at 1 hz^H^H^Hfull rate)
    if ( flush_counter >= 0 /*HEARTBEAT_HZ*/ ) {
	datalog_prof.start();
	flush_counter = 0;
	if ( log_to_file ) {
	    flush_data();
	}
	datalog_prof.stop();
    }

    debug7.stop();

    main_prof.stop();
}


//
// main ...
//
int main( int argc, char **argv )
{
    int iarg;

    // Parse command line: Pass #1 to scan for a custom config root
    // and python module path on command line
    string root = "./config";
    string python_path = "";
    for ( iarg = 1; iarg < argc; iarg++ ) {
	if ( !strcmp(argv[iarg], "--config" )  ) {
	    ++iarg;
	    root = argv[iarg];
	} else if ( !strcmp(argv[iarg], "--python_path" )  ) {
	    ++iarg;
	    python_path = argv[iarg];
	}
    }

    // destroy things in the correct order
    // atexit(AuraPythonCleanup);
    
    // initialize network library
    netInit( NULL, NULL );

    // initialize python
    AuraPythonInit(argc, argv, python_path.c_str());

    // initialize properties
    pyPropsInit();
    status_node = pyGetNode("/status", true);
    status_node.setDouble("frame_time", get_Time());

    // initialize profiling names
    imu_prof.set_name("imu");
    gps_prof.set_name("gps");
    air_prof.set_name("airdata");
    pilot_prof.set_name("pilot");
    filter_prof.set_name("filter");
    control_prof.set_name("control");
    route_mgr_prof.set_name("filter");
    health_prof.set_name("health");
    datalog_prof.set_name("datalogger");
    sync_prof.set_name("sync");
    main_prof.set_name("main");
    
    // debugging
    debug1.set_name("debug1 (var updates)");
    debug2.set_name("debug2 (inputs)");
    debug2a.set_name("debug2a (IMU)");
    debug2b.set_name("debug2b (AirData)");
    debug2c.set_name("debug2c (GPS)");
    debug2d.set_name("debug2d (Pilot)");
    debug3.set_name("debug3 (filter+nav)");
    debug4.set_name("debug4 (console)");
    debug5.set_name("debug5 (telnet)");
    debug6.set_name("debug6 (ap+actuator)");
    debug7.set_name("debug7 (logging)");

    if ( display_on ) {
	printf("Main clock resolution:\n");
	print_Time_Resolution();
    }
    
    // load master config file
    SGPath master( root );
    master.append( "main.xml" );
    try {
	pyPropertyNode props = pyGetNode("/", true);
        readXML( master.c_str(), &props);
        printf("Loaded configuration from %s\n", master.c_str());
	//writeXML( "debug.xml", &props);
	props.pretty_print();
	pyPropertyNode config_node = pyGetNode("/config");
	config_node.setString("root-path", root.c_str());
    } catch (const sg_exception &exc) {
        printf("\n");
        printf("*** Cannot load master config file: %s\n", master.c_str());
	printf("*** \n%s\n***\n", exc.getFormattedMessage().c_str());
        printf("\n");
        sleep(1);
    }

    // extract configuration values from the property tree (which is
    // now populated with the master config.xml data.  Do this before
    // the command line processing so that any options specified on
    // the command line will override what is in the config.xml file.

    pyPropertyNode p;

    p = pyGetNode("/config/logging", true);
    if ( p.hasChild("path") ) {
	log_path.set( p.getString("path") );
    }
    if ( p.hasChild("enable") ) {
	log_to_file = p.getBool("enable");
    }
    printf("log path = %s enabled = %d\n", log_path.c_str(), log_to_file);

    p = pyGetNode("/config/telnet", true);
    if ( p.hasChild("enable") ) {
	enable_telnet = p.getBool("enable");
    }
    if ( enable_telnet ) {
	if ( p.hasChild("port") ) {
	    telnet = new UGTelnet( p.getLong("port") );
	    telnet->open();
	} else {
	    printf("No telnet port defined, disabling telnet interface\n");
	    enable_telnet = false;
	}
    }

    p = pyGetNode("/config/pointing", true);
    if ( p.hasChild("enable") ){
	printf("Pointing = %s\n", p.getString("enable").c_str());
	enable_pointing = p.getBool("enable");
	printf("Pointing = %d\n", enable_pointing);
    }

    p = pyGetNode("/config", true);
    if ( p.hasChild("gps_timeout_sec") ) {
	gps_timeout_sec = p.getDouble("gps_timeout_sec");
    }
    printf("gps timeout = %.1f\n", gps_timeout_sec);

    p = pyGetNode("/config/mission", true);
    if ( p.hasChild("enable") ) {
	enable_mission = p.getBool("enable");
    }

    // Parse the command line: pass #2 allows command line options to
    // override config file options
    for ( iarg = 1; iarg < argc; iarg++ ) {
        if ( !strcmp(argv[iarg], "--log-dir" )  ) {
            ++iarg;
            log_path.set( argv[iarg] );
            log_to_file = true;
        } else if ( !strcmp(argv[iarg], "--remote-link" )  ) {
            ++iarg;
            if ( !strcmp(argv[iarg], "on") ) remote_link_on = true;
            if ( !strcmp(argv[iarg], "off") ) remote_link_on = false;
        } else if ( !strcmp(argv[iarg],"--display") ) {
            ++iarg;
            if ( !strcmp(argv[iarg], "on") ) display_on = true;
            if ( !strcmp(argv[iarg], "off") ) display_on = false;
        } else if ( !strcmp(argv[iarg], "--config" )  ) {
   	    // considered earlier in first pass
            ++iarg;
        } else if ( !strcmp(argv[iarg], "--python_path" )  ) {
   	    // considered earlier in first pass
            ++iarg;
        } else if ( !strcmp(argv[iarg], "--help") ) {
            usage(argv[0]);
        } else {
            printf("Unknown option \"%s\"\n", argv[iarg]);
            usage(argv[0]);
        }
    }

    // initialize required aura-core structures
    AuraCoreInit();

    // open remote link if requested
    if ( remote_link_on ) {
        remote_link_init();
    }

    // open logging files if requested
    if ( log_to_file ) {
        if ( !logging_init() ) {
            printf("Warning: error opening one or more data files, logging disabled\n");
            log_to_file = false;
        }
    }

    // Initialize communication with the selected IMU
    IMU_init();

    // Initialize communication with the selected air data sensor
    AirData_init();

    // Initialize communication with the selected GPS
    GPS_init();

    // Initialize communication with pilot input sensor
    PilotInput_init();

    // Initialize any defined filter modules
    Filter_init();

    // init system health and status monitor
    health_init();

    // init payload manager
    payload_mgr.init();

    // if ( enable_pointing ) {
    // 	// initialize pointing module
    // 	ati_pointing_init();
    // }

    // initialize the autopilot
    control_init();

    // initialize the actuators
    Actuator_init();

    if ( enable_cas ) {
	// initialize the cas system
	cas.init();
    }

    // intialize random number generator
    srandom( time(NULL) );

    printf("Everything inited ... ready to run\n");

    while ( true ) {
	main_work_loop();
    }

    // close and exit
    Filter_close();
    IMU_close();
    GPS_close();
    AirData_close();
    PilotInput_close();
    // if ( enable_pointing ) {
    // 	ati_pointing_close();
    // }
    payload_mgr.close();
    control_close();
    Actuator_close();
}


