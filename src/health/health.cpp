// System health/status monitoring module


#include <pyprops.h>

#include <stdio.h>

#include "include/globaldefs.h"
#include "util/myprof.h"

#include "health.h"
#include "loadavg.h"

void health_t::init() {
    loadavg_init();
}

void health_t::update() {
    health_prof.start();
    loadavg_update();
    health_prof.stop();
}

// global shared instance
health_t health;
