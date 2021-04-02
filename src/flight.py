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

from comms import display, logging, remote_link, telnet
from drivers import pilot_helper
from mission import mission_mgr
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

# #include "control/actuators.h"
# #include "control/cas.h"
# #include "control/control.h"
# #include "filters/filter_mgr.h"
# #include "health/health.h"
# #include "init/globals.h"
# #include "util/netSocket.h"	// netInit()
# #include "util/sg_path.h"

# configuration settings

# static bool enable_cas     = false;   // cas module enabled/disabled
# static bool enable_pointing = false;  // pan/tilt pointing module

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
    
    # read an entire frame of sensors data
    myprof.driver_prof.start()
    dt = drivers.read()
    myprof.driver_prof.stop()
    
    myprof.main_prof.start()
    
    status_node.setFloat("frame_time", imu_node.getFloat("timestamp"))
    status_node.setFloat("dt", dt)
    
    # extra sensor processing section
    myprof.helper_prof.start()
    airdata.update()
    gps.update(display_on)    # computes gps age (and sets host clock)
    pilot.update()            # log auto/manual changes, transient reduction
    # check gps data age.  The nav filter continues to run, but the
    # results are marked as NotValid if the most recent gps data
    # becomes too old.
    if gps.gps_age() > gps_timeout_sec:
        status_node.setString("navigation", "invalid")
    myprof.helper_prof.stop()

#     //
#     // State Estimation section
#     //
#     Filter_update();

#     //
#     // Core Flight Control section
#     //

#     if ( enable_cas ) {
# 	cas.update();
#     }

#     control.update( dt );

#     // convert logical flight controls into physical actuator outputs
#     actuators.update();

    # write commands back to drivers
    drivers.write()

    # send any extra commands (like requests to recalibrate something)
    drivers.send_commands()
    
    # check for incoming command data
    remote_link.command()

    # Read commands from telnet interface
    telnet.update()

#     // if ( enable_pointing ) {
#     // 	// Update pointing module
#     // 	ati_pointing_update( dt );
#     // }

    # Mission and Task section
    myprof.mission_prof.start()
    mission.update(dt)
    myprof.mission_prof.stop()

#     // health status
#     health.update();

    # sensor summary display @ 2 second interval
    if display_on and timer.get_pytime() >= display_timer + 2:
        display_timer += 2
        display.status_summary()
        myprof.driver_prof.stats()
        myprof.helper_prof.stats()
        myprof.filter_prof.stats()
        myprof.mission_prof.stats()
        myprof.control_prof.stats()
        myprof.health_prof.stats()
        myprof.datalog_prof.stats()
        myprof.main_prof.stats()

    # flush of log stream
    myprof.datalog_prof.start()
    logging.update()
    myprof.datalog_prof.stop()

    # generate needed messages and dribble pending bytes down the serial port
    remote_link.update()

    myprof.main_prof.stop()


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
mission = mission_mgr.MissionMgr()

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

# initalize communication modules first thing after loading config
logging.init()
remote_link.init()
telnet.init()

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
mission.init()

    
# log the master config tree
logging.write_configs()

print("Everything is initized ... enter main work loop.");

while True:
    main_work_loop()

# close and exit
# Filter_close()
logging.close()



