// System health/status monitoring module


#include <pyprops.h>

#include <stdio.h>

#include "include/globaldefs.h"

#include "health.h"
#include "loadavg.h"

void health_init() {
    loadavg_init();
}

void health_update() {
    loadavg_update();
}
