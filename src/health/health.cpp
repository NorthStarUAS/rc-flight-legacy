// System health/status monitoring module


#include <stdio.h>

#include "include/globaldefs.h"

#include "comms/console_link.h"
#include "control/route_mgr.hxx"
#include "props/props.hxx"
#include "util/timing.h"

#include "health.h"
#include "loadavg.h"
//#include "sgbatmon.h"


struct health healthpacket;

static SGPropertyNode *ap_roll;
static SGPropertyNode *ap_hdg;
static SGPropertyNode *ap_pitch;
static SGPropertyNode *ap_climb;
static SGPropertyNode *ap_altitude;
static SGPropertyNode *ground_ref;
static SGPropertyNode *ap_agl;


bool health_init() {
    loadavg_init();
    //sgbatmon_init();

    ap_roll = fgGetNode("/autopilot/internal/target-roll-deg", true);
    ap_hdg = fgGetNode( "/autopilot/settings/true-heading-deg", true );
    ap_pitch = fgGetNode( "/autopilot/settings/target-pitch-deg", true );
    ap_climb = fgGetNode("/autopilot/internal/target-climb-rate-fps", true);
    ap_altitude = fgGetNode( "/autopilot/settings/target-altitude-ft", true );
    ground_ref = fgGetNode( "/position/ground-altitude-pressure-m", true );
    ap_agl = fgGetNode( "/autopilot/settings/target-agl-ft", true );

    // set initial values
    healthpacket.command_sequence = 0;
    healthpacket.target_waypoint = 0;

    return true;
}


bool health_update() {
    static int wp_index = 0;

    healthpacket.time = get_Time();

    healthpacket.target_roll_deg = ap_roll->getDoubleValue();
    healthpacket.target_heading_deg = ap_hdg->getDoubleValue();
    healthpacket.target_pitch_deg = ap_pitch->getDoubleValue();
    healthpacket.target_climb_fps = ap_climb->getDoubleValue();
    /* healthpacket.target_altitude_ft = ap_altitude->getDoubleValue(); */
    healthpacket.target_altitude_ft
        = ground_ref->getDoubleValue() * SG_METER_TO_FEET
          + ap_agl->getDoubleValue();

    loadavg_update();
    //sgbatmon_update();

    // send each waypoint, then home location (with wp_index = 0)
    int size = route_mgr.size();
    if ( size > 0 && wp_index < size ) {
        SGWayPoint wp = route_mgr.get_waypoint( wp_index );
        healthpacket.wp_lon = wp.get_target_lon();
        healthpacket.wp_lat = wp.get_target_lat();
        healthpacket.wp_index = wp_index + 1000000*size;
        wp_index++;
    } else {
        SGWayPoint home = route_mgr.get_home();
        healthpacket.wp_lon = home.get_target_lon();
        healthpacket.wp_lat = home.get_target_lat();
        healthpacket.wp_index = 0;
        wp_index = 0;
    }

    return true;
}


void health_update_command_sequence( int sequence ) {
    healthpacket.command_sequence = sequence;
}


void health_update_target_waypoint( int index ) {
    healthpacket.target_waypoint = index;
}
