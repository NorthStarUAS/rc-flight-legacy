// System health/status monitoring module


#include <stdio.h>

#include "include/globaldefs.h"

#include "comms/console_link.h"
#include "util/timing.h"

#include "health.h"
#include "loadavg.h"
//#include "sgbatmon.h"


struct health healthpacket;


bool health_init() {
    loadavg_init();
    //sgbatmon_init();

    // set initial values
    healthpacket.command_sequence = 0;
    healthpacket.target_waypoint = 0;

    return true;
}


bool health_update() {
    healthpacket.time = get_Time();

    loadavg_update();
    //sgbatmon_update();

    return true;
}


void health_update_command_sequence( int sequence ) {
    healthpacket.command_sequence = sequence;
}


void health_update_target_waypoint( int index ) {
    healthpacket.target_waypoint = index;
}
