// System health/status monitoring module


#include <pyprops.hxx>

#include <stdio.h>

#include "include/globaldefs.h"

#include "comms/aura_messages.h"
#include "comms/logging.hxx"
#include "comms/remote_link.hxx"
#include "init/globals.hxx"
#include "util/timing.h"

#include "health.hxx"
#include "loadavg.hxx"


static pyPropertyNode remote_link_node;
static pyPropertyNode logging_node;
static pyPropertyNode status_node;
static pyPropertyNode power_node;


bool health_init() {
    loadavg_init();

    // initialize comm nodes
    remote_link_node = pyGetNode("/config/remote_link", true);
    logging_node = pyGetNode("/config/logging", true);
    status_node = pyGetNode("/status", true);
    power_node = pyGetNode("/sensors/power", true);

    return true;
}


bool health_update() {
    loadavg_update();

    message::system_health_v5_t health;
    health.index = 0;
    health.timestamp_sec = status_node.getDouble("frame_time");
    health.system_load_avg = status_node.getDouble("system_load_avg");
    health.avionics_vcc = power_node.getDouble("avionics_vcc");
    health.main_vcc = power_node.getDouble("main_vcc");
    health.cell_vcc = power_node.getDouble("cell_vcc");
    health.main_amps = power_node.getDouble("main_amps");
    health.total_mah = power_node.getDouble("total_mah");
    health.pack();
    remote_link->send_message( health.id, health.payload, health.len );
    logging->log_message( health.id, health.payload, health.len );

    return true;
}
