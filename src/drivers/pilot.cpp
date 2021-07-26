/**
 * \file: pilot.cpp
 *
 * Pilot input helper
 *
 * Copyright (C) 2010-2020 Curtis L. Olson curtolson@flightgear.org
 *
 */


#include <pybind11/pybind11.h>
namespace py = pybind11;

#include <stdio.h>
#include <string.h>

#include "init/globals.h"

#include "pilot.h"

void pilot_helper_t::init(DocPointerWrapper d) {
    PropertyNode("/").set_Document(d);

    pilot_node = PropertyNode("/sensors/pilot_input", true);
    flight_node = PropertyNode("/controls/flight", true);
    engine_node = PropertyNode("/controls/engine", true);
    ap_node = PropertyNode("/autopilot", true);
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
        flight_node.setFloat( "aileron", pilot_node.getFloat("aileron") );
        flight_node.setFloat( "elevator", pilot_node.getFloat("elevator") );
        engine_node.setFloat( "throttle", pilot_node.getFloat("throttle") );
        flight_node.setFloat( "rudder", pilot_node.getFloat("rudder") );
        flight_node.setFloat( "flaps", pilot_node.getFloat("flaps") );
    }
}

PYBIND11_MODULE(pilot_helper, m) {
    py::class_<pilot_helper_t>(m, "pilot_helper")
        .def(py::init<>())
        .def("init", &pilot_helper_t::init)
        .def("update", &pilot_helper_t::update)
    ;
}
