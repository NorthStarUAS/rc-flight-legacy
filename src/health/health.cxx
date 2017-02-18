// System health/status monitoring module


#include "python/pyprops.hxx"

#include <stdio.h>

#include "include/globaldefs.h"

#include "comms/logging.hxx"
#include "comms/remote_link.hxx"
#include "init/globals.hxx"
#include "util/timing.h"

#include "health.hxx"
#include "loadavg.hxx"


static pyPropertyNode remote_link_node;
static pyPropertyNode logging_node;


bool health_init() {
    loadavg_init();

    // initialize comm nodes
    remote_link_node = pyGetNode("/config/remote_link", true);
    logging_node = pyGetNode("/config/logging", true);

    return true;
}


bool health_update() {
    loadavg_update();

    uint8_t buf[256];
    int size = packer->pack_health( 0, buf );

    if ( remote_link_on ) {
        remote_link_health( buf, size, remote_link_node.getLong("health_skip") );
    }

    logging->log_health( buf, size );

    return true;
}
