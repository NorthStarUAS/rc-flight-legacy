/**
 * \file: pilot_mgr.cpp
 *
 * Front end management interface for reading pilot input.
 *
 * Copyright (C) 2010 - Curtis L. Olson curtolson@flightgear.org
 *
 */


#include <pyprops.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <sstream>
#include <string>
#include <vector>
using std::ostringstream;
using std::string;
using std::vector;

#include "comms/aura_messages.h"
#include "comms/logging.h"
#include "comms/remote_link.h"
#include "include/globaldefs.h"
#include "init/globals.h"
#include "util/myprof.h"

#include "APM2.h"
#include "Aura3/Aura3.h"

#include "pilot_mgr.h"

//
// Global variables
//

// property nodes
static pyPropertyNode pilot_node;
static pyPropertyNode flight_node;
static pyPropertyNode engine_node;
static pyPropertyNode ap_node;
static vector<pyPropertyNode> sections;
static vector<pyPropertyNode> outputs;

void PilotInput_init() {
    pilot_node = pyGetNode("/sensors/pilot_input", true);
    flight_node = pyGetNode("/controls/flight", true);
    engine_node = pyGetNode("/controls/engine", true);
    ap_node = pyGetNode("/autopilot", true);
}


bool PilotInput_update() {
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
    return true;
}
