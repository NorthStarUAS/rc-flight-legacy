#!/usr/bin/env python3

#
# flight.py - top level "main" program for the Rice Creek UAS autopilot system
#
# Written by Curtis Olson, curtolson <at> flightgear <dot> org.
# Started 2007.
# 
# This code is released under the terms of the MIT open-source license

import argparse
import os

from props import getNode, root
import props_json

from rcUAS import driver_mgr
from rcUAS import airdata_helper, gps_helper

from comms import display
from drivers import pilot_helper
from util import myprof, timer

# #include <python_sys.h>
# #include <pyprops.h>

# #include <stdio.h>
# #include <sys/types.h>

# #include <sys/stat.h>
# #include <fcntl.h>
# #include <sys/time.h>
# #include <stdlib.h>
# #include <string.h>
# #include <sys/resource.h>
# #include <unistd.h>

# #include <string>
# using std::string;

# #include "include/aura_config.h"

# #include "comms/logging.h"
# #include "comms/remote_link.h"
# #include "control/actuators.h"
# #include "control/cas.h"
# #include "control/control.h"
# #include "drivers/pilot.h"
# #include "filters/filter_mgr.h"
# #include "health/health.h"
# #include "init/globals.h"
# #include "util/myprof.h"
# #include "util/netSocket.h"	// netInit()
# #include "util/sg_path.h"
# #include "util/timing.h"

# //
# // Configuration settings
# //

# static const int HEARTBEAT_HZ = 100;  // master clock rate

# static bool enable_mission = true;    // mission mgr module enabled/disabled
# static bool enable_cas     = false;   // cas module enabled/disabled
# static bool enable_pointing = false;  // pan/tilt pointing module
# static double gps_timeout_sec = 9.0;  // nav algorithm gps timeout

# // property nodes
# static pyPropertyNode imu_node;
# static pyPropertyNode status_node;
# static pyPropertyNode comms_node;

parser = argparse.ArgumentParser(description="Rice Creak UAS main flight code")
parser.add_argument("--config", required=True, help="path to config tree")
parser.add_argument("--verbose", action="store_true", help="enable additional console verbocity")
args = parser.parse_args()

display_timer = timer.get_pytime()
def main_work_loop():
    # update display_on variable
    global display_on
    global display_timer
    display_on = comms_node.getBool("display_on");
    
#     // read the sensors until we receive an IMU packet
#     sync_prof.start();
#     double dt = 0.0;

    myprof.driver_prof.start()
    dt = drivers.read()
    myprof.driver_prof.stop()
    
#     status_node.setDouble("frame_time", imu_node.getDouble( "timestamp" ));
#     status_node.setDouble("dt", dt);
#     sync_prof.stop();
    
#     main_prof.start();
    
#     static int count = 0;
#     count++;
#     // printf ("timer expired %d times\n", count);

    # extra sensor processing section
    myprof.airdata_prof.start()
    airdata.update()
    myprof.airdata_prof.stop()

    gps.update(display_on)    # computes gps age (and sets host clock)
    pilot.update()            # log auto/manual changes, transient reduction

#     //
#     // State Estimation section
#     //
#     Filter_update();

    # check gps data age.  The nav filter continues to run, but the
    # results are marked as NotValid if the most recent gps data
    # becomes too old.
    if gps.gps_age() > gps_timeout_sec:
        status_node.setString("navigation", "invalid")

#     //
#     // Core Flight Control section
#     //

#     if ( enable_cas ) {
# 	cas.update();
#     }

#     control.update( dt );

#     // convert logical flight controls into physical actuator outputs
#     actuators.update();

#     driver_mgr.write();

#     // send any extra commands (like requests to recalibrate something)
#     driver_mgr.send_commands();
    
#     //
#     // External Command section
#     //

#     // check for incoming command data
#     remote_link->command();

#     //
#     // Read commands from telnet interface
#     //
#     telnet->update(0);

#     // if ( enable_pointing ) {
#     // 	// Update pointing module
#     // 	ati_pointing_update( dt );
#     // }

#     //
#     // Mission and Task section
#     //

#     mission_prof.start();
#     if ( enable_mission ) {
# 	mission_mgr->update(dt);
#     }
#     mission_prof.stop();

#     // health status
#     health.update();

    # sensor summary display @ 2 second interval
    if display_on and timer.get_pytime() >= display_timer + 2:
        display_timer += 2
        display.status_summary()
        myprof.airdata_prof.stats()
        myprof.driver_prof.stats()
        myprof.filter_prof.stats()
        myprof.mission_prof.stats()
        myprof.control_prof.stats()
        myprof.health_prof.stats()
        myprof.datalog_prof.stats()
        myprof.sync_prof.stats()
        myprof.main_prof.stats()

#     // flush of logging stream (update at full rate)
#     if ( true ) {
# 	datalog_prof.start();
#         logging->update();
# 	datalog_prof.stop();
#     }

#     //
#     // Remote telemetry section
#     //

#     // generate needed messages and dribble pending bytes down the serial port
#     remote_link->update();

#     main_prof.stop();
# }


# Initialization Section

display_on = False

comms_node = getNode("/comms", True)
status_node = getNode("/status", True)
status_node.setFloat("frame_time", 0.0)
imu_node = getNode("/sensors/imu", True)

drivers = driver_mgr.driver_mgr()
airdata = airdata_helper.airdata_helper()
gps = gps_helper.gps_helper()
pilot = pilot_helper.pilot_helper()

# load master config file
config_file = os.path.join( args.config, "main.json")
result = props_json.load(config_file, root)
if result:
    print("Loaded master configuration file:", config_file)
    if args.verbose:
        root.pretty_print()
    config_node = getNode("/config")
    config_node.setString("path", args.config)
else:
    print("*** Cannot load master config file:", config_file)
    print()
    print("Cannot continue without a valid configuration, sorry.")
    exit(-1)

#     // extract configuration values from the property tree (which is
#     // now populated with the master config.xml data.  Do this before
#     // the command line processing so that any options specified on
#     // the command line will override what is in the config.xml file.

#     pyPropertyNode p;

#     p = pyGetNode("/config/pointing", true);
#     if ( p.hasChild("enable") ){
# 	printf("Pointing = %s\n", p.getString("enable").c_str());
# 	enable_pointing = p.getBool("enable");
# 	printf("Pointing = %d\n", enable_pointing);
#     }

if config_node.hasChild("gps_timeout_sec"):
    gps_timeout_sec = config_node.getFloat("gps_timeout_sec")
    print("gps timeout = %.1f" % gps_timeout_sec)

#     p = pyGetNode("/config/mission", true);
#     if ( p.hasChild("enable") ) {
# 	enable_mission = p.getBool("enable");
#     }

if args.verbose:
    comms_node.setBool("display_on", True)

#     // Parse the command line: pass #2 allows command line options to
#     // override config file options
#     for ( iarg = 1; iarg < argc; iarg++ ) {
#         } else if ( !strcmp(argv[iarg], "--config" )  ) {
#    	    // considered earlier in first pass
#             ++iarg;
#         } else if ( !strcmp(argv[iarg], "--python_path" )  ) {
#    	    // considered earlier in first pass
#             ++iarg;
#         } else if ( !strcmp(argv[iarg], "--help") ) {
#             usage(argv[0]);
#         } else {
#             printf("Unknown option \"%s\"\n", argv[iarg]);
#             usage(argv[0]);
#         }
#     }

#     // initialize required aura-core structures
#     AuraCoreInit();

drivers.init()

# data helpers
airdata.init()
gps.init()
pilot.init()

#     // Initialize any defined filter modules
#     Filter_init();

#     // init system health and status monitor
#     health.init();

#     // if ( enable_pointing ) {
#     // 	// initialize pointing module
#     // 	ati_pointing_init();
#     // }

#     // initialize the autopilot
#     control.init();

#     // initialize the actuator output
#     actuators.init();

#     if ( enable_cas ) {
# 	// initialize the cas system
# 	cas.init();
#     }

#     // intialize random number generator
#     srandom( time(NULL) );

#     // log the master config tree
#     logging->write_configs();
    
print("Everything is initized ... enter main work loop.");

while True:
    main_work_loop();

#     // close and exit
#     Filter_close();
#     logging->close();
# }


