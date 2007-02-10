// System health/status monitoring module

#include "comms/console_link.h"

#include "health.h"
#include "sgbatmon.h"


struct health healthpacket;


bool health_init() {
    sgbatmon_init();
    return true;
}


bool health_update() {
    sgbatmon_update();

    if ( console_link_on ) {
      console_link_health( &healthpacket );
    }

    return true;
}
