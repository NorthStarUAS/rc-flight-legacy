// System health/status monitoring module

#include "comms/console_link.h"
#include "util/timing.h"

#include "health.h"
#include "loadavg.h"
#include "sgbatmon.h"


struct health healthpacket;


bool health_init() {
    loadavg_init();
    sgbatmon_init();

    return true;
}


bool health_update() {
    healthpacket.time = get_Time();

    loadavg_update();
    sgbatmon_update();

    if ( console_link_on ) {
      console_link_health( &healthpacket );
    }

    return true;
}
