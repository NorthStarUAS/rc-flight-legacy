// System health/status monitoring module


#include <stdio.h>

#include "include/globaldefs.h"

#include "comms/console_link.h"
#include "comms/logging.h"
#include "control/route_mgr.hxx"
#include "props/props.hxx"
#include "util/timing.h"

#include "health.h"
#include "loadavg.h"


bool health_init() {
    loadavg_init();

    return true;
}


bool health_update() {
    // static int wp_index = 0;

    /*
    healthpacket.target_altitude_ft
        = ground_ref->getDoubleValue() * SG_METER_TO_FEET
        + pressure_error_m_node->getDoubleValue() * SG_METER_TO_FEET
          + ap_agl->getDoubleValue();
    */

    loadavg_update();

#if 0
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
#endif

    return true;
}
