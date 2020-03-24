/**
 * \file: pilot.cpp
 *
 * Pilot input helper
 *
 * Copyright (C) 2010-2020 Curtis L. Olson curtolson@flightgear.org
 *
 */


#include <pyprops.h>

#include <stdio.h>
#include <string.h>

#include "init/globals.h"

#include "pilot.h"

void pilot_helper_t::init() {
    pilot_node = pyGetNode("/sensors/pilot_input", true);
    flight_node = pyGetNode("/controls/flight", true);
    engine_node = pyGetNode("/controls/engine", true);
    ap_node = pyGetNode("/autopilot", true);
}

void pilot_helper_t::update() {
    // log receiver fail safe changes
    static bool last_fail_safe = false;
    if ( pilot_node.getBool("fail_safe") != last_fail_safe ) {
        char buf[128];
        snprintf( buf, 32, "Receiver fail safe = %d",
                  pilot_node.getBool("fail_safe") );
        events->log("Aura", buf );
        last_fail_safe = pilot_node.getBool("fail_safe");
    }
        
    // Only in manual mode, do copy the pilot inputs to the main
    // AP outputs.  This puts the pilot inputs in a standard place
    // and allows the AP to seed it's components with trimmed
    // values and improve continuity when switching from manual to
    // AP mode.
    if ( ! ap_node.getBool("master_switch") ) {
        flight_node.setDouble( "aileron", pilot_node.getDouble("aileron") );
        flight_node.setDouble( "elevator", pilot_node.getDouble("elevator") );
        engine_node.setDouble( "throttle", pilot_node.getDouble("throttle") );
        flight_node.setDouble( "rudder", pilot_node.getDouble("rudder") );
        flight_node.setDouble( "flaps", pilot_node.getDouble("flaps") );
    }
}

// global shared instance
pilot_helper_t pilot_helper;
