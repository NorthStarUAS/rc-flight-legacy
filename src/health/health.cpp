// System health/status monitoring module


#include <pyprops.h>

#include <stdio.h>

#include "include/globaldefs.h"

#include "comms/aura_messages.h"
#include "comms/logging.h"
#include "comms/remote_link.h"
#include "init/globals.h"
#include "util/timing.h"

#include "health.h"
#include "loadavg.h"


static pyPropertyNode remote_link_node;
static pyPropertyNode logging_node;
static pyPropertyNode status_node;
static pyPropertyNode power_node;


void health_init() {
    loadavg_init();
}


void health_update() {
    loadavg_update();
}
