#!/usr/bin/env python3

#
# flight.py - top level "main" program for the Rice Creek UAS autopilot system
#
# Written by Curtis Olson, curtolson <at> flightgear <dot> org.
# Started 2007.
# 
# This code is released under the terms of the MIT open-source license.

import argparse
import os
import traceback

from props import getNode, root
import props_json

# C++ modules
from rcUAS import actuator_mgr, control_mgr, driver_mgr, filter_mgr
from rcUAS import airdata_helper, gps_helper

# Pure python modules
from comms import display, logging, remote_link, telnet
from control import navigation
from drivers import pilot_helper
from health import health
from mission import mission_mgr
from util import myprof, timer

parser = argparse.ArgumentParser(description="Rice Creak UAS flight code")
parser.add_argument("--config", required=True, help="path to config tree")
parser.add_argument("--verbose", action="store_true", help="enable additional console verbocity")
args = parser.parse_args()

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

# shared property nodes
comms_node = getNode("/comms", True)
imu_node = getNode("/sensors/imu", True)
status_node = getNode("/status", True)
status_node.setFloat("frame_time", 0.0)

# create singleton class instances
actuators = actuator_mgr.actuator_mgr()
control = control_mgr.control_mgr()
drivers = driver_mgr.driver_mgr()
airdata = airdata_helper.airdata_helper()
gps = gps_helper.gps_helper()
pilot = pilot_helper.pilot_helper()

# configuration options
if args.verbose:
    comms_node.setBool("display_on", True)

if config_node.hasChild("gps_timeout_sec"):
    gps_timeout_sec = config_node.getFloat("gps_timeout_sec")
    print("gps timeout = %.1f" % gps_timeout_sec)

# module initialization
def init():
    # communication modules
    logging.init()
    remote_link.init()
    telnet.init()

    # hardware
    drivers.init()

    # sensor processing helpers
    airdata.init()
    gps.init()
    pilot.init()

    # health monitor
    health.init()

    # sensor fusion, ins/gns, ekf, wind
    filter_mgr.init()

    # if enable_pointing:
    #     ati_pointing_init()

    # autopilot, flight control modules
    control.init()
    navigation.init()

    # effectors
    actuators.init()

    # mission and task system
    mission_mgr.init()

    # save the master config tree with the flight data
    logging.write_configs()

    print("Initialization complete.");

display_timer = timer.get_pytime()
def update():
    # update display_on variable
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
    gps.update(display_on)    # computes gps age (optionally sets host clock)
    pilot.update()            # log auto/manual changes, transient reduction
    
    # check gps data age.  The nav filter continues to run, but the
    # results are marked as invalid if the most recent gps data
    # becomes too old.
    if gps.gps_age() > gps_timeout_sec:
        status_node.setString("navigation", "invalid")
    myprof.helper_prof.stop()

    # State estimation (ins/gns ekf)
    myprof.filter_prof.start()
    filter_mgr.update();
    myprof.filter_prof.stop()

    # flight control
    myprof.control_prof.start()
    navigation.update(dt)
    control.update(dt)
    myprof.control_prof.stop()

    # convert logical flight controls into physical actuator outputs
    actuators.update()

    # write effector commands back to drivers
    drivers.write()

    # send any extra commands (like requests to recalibrate something)
    drivers.send_commands()
    
    # check for incoming operator commands
    remote_link.command()

    # read commands from telnet interface
    telnet.update()

    # if enable_pointing:
    #     ati_pointing_update( dt )

    # mission and task section
    myprof.mission_prof.start()
    mission_mgr.update(dt)
    myprof.mission_prof.stop()

    # health status
    health.update()

    # flush of log stream
    myprof.datalog_prof.start()
    logging.update()
    myprof.datalog_prof.stop()

    # generate needed messages and dribble pending bytes down the serial port
    remote_link.update()

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

    myprof.main_prof.stop()

# Here is the top level main program.  In arduino style, we call the
# init() function once and then loop the update() function forever.
#
# Note that the driver module is the heartbeat of the system so that
# the data rate from the driver module dictates the main loop update
# rate.
init()
print("Entering main update loop...")
while True:
    try:
        update()
    except Exception as e:
        print("Main loop encountered an exception:", str(e))
        traceback.print_exc()

# close and exit
filter_mgr.close()
logging.close()
