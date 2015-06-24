// System health/status monitoring module


#include <stdio.h>

#include "include/globaldefs.h"

#include "comms/logging.h"
#include "comms/packetizer.hxx"
#include "comms/remote_link.h"
#include "init/globals.hxx"
#include "props/props.hxx"
#include "util/timing.h"

#include "health.h"
#include "loadavg.h"


static SGPropertyNode *input_vcc_node = NULL;
static SGPropertyNode *health_console_skip = NULL;
static SGPropertyNode *health_logging_skip = NULL;


bool health_init() {
    loadavg_init();

    input_vcc_node = fgGetNode("/sensors/APM2/board-vcc", true);

    // initialize comm nodes
    health_console_skip = fgGetNode("/config/remote-link/health-skip", true);
    health_logging_skip = fgGetNode("/config/logging/health-skip", true);

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

    if ( remote_link_on || log_to_file ) {
	uint8_t buf[256];
	int size = packetizer->packetize_health( buf );

	if ( remote_link_on ) {
	    remote_link_health( buf, size, health_console_skip->getIntValue() );
	}

	if ( log_to_file ) {
	    log_health( buf, size, health_logging_skip->getIntValue() );
	}
    }

    return true;
}
