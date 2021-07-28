/**
 * \file: actuators.cpp
 *
 * convert logical flight controls into physical actuator outputs
 *
 * Copyright (C) 2009-2020 Curtis L. Olson curtolson@flightgear.org
 */

#include <pybind11/pybind11.h>
namespace py = pybind11;

#include <stdio.h>

#include <string>
using std::string;

#include "util/timing.h"
#include "actuators.h"

void actuators_t::init(DocPointerWrapper d) {
    PropertyNode("/").set_Document(d);
 
    // bind properties
    flight_node = PropertyNode( "/controls/flight" );
    engine_node = PropertyNode( "/controls/engine" );
    excite_node = PropertyNode( "/task/excite" );
    pilot_node = PropertyNode( "/sensors/pilot_input" );
    act_node = PropertyNode( "/actuators" );
    ap_node = PropertyNode( "/autopilot" );
}

void actuators_t::update() {
    // set time stamp for logging
    act_node.setDouble( "timestamp", get_Time() );

    float aileron = flight_node.getFloat("aileron");
    act_node.setFloat( "aileron", aileron );

    float elevator = flight_node.getFloat("elevator");
    act_node.setFloat( "elevator", elevator );

    // rudder
    float rudder = flight_node.getFloat("rudder");
    act_node.setFloat( "rudder", rudder );

    double flaps = flight_node.getFloat("flaps");
    act_node.setFloat("flaps", flaps );

    double gear = flight_node.getFloat("gear");
    act_node.setFloat("gear", gear );

    // CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!!
    // CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!!

    // Placing the engine throttle under autopilot control requires
    // EXTREME care!!!!

    // Propellers are constructed of sharp knife-like material.
    // Electric motors don't quit and give up if they encounter intial
    // resistance.  Severe injuries to hand or face or any other body
    // part in the vicinity of the motor or prop can occur at any
    // time.

    // Care must be taken during initial setup, and then from that
    // point on during all operational, testing, and ground handling
    // phases.  Extreme vigilance must always be maintianed at all
    // times (especially if the autopilot has control of the
    // throttle.)

    // I cannot stress this point enough!!!  One nanosecond of
    // distraction or loss of focus can result in severe lifelong
    // injury or death!  Do not take your fingers or face for granted.
    // Always maintain utmost caution and correct safety procedures to
    // ensure safe operation with a throttle enabled UAS:

    // 1. Never put your fingers or any other body part in the
    // vicinity or path of the propellor.

    // 2. When the prop is moving (i.e. power test on the ground)
    // always stay behind the prop arc.  If a blade shatters it will
    // shoot outwards and forwards and you never want to be in the
    // path of a flying knife.

    // 3. Always stay behind the aircraft.  If the engine
    // inadvertantly powers up or goes from idle to full throttle, the
    // aircraft could be propelled right at you.

    // Safety is ultimately the responsibility of the operator at the
    // field.  Never put yourself or helpers or spectators in a
    // position where a moment of stupidity will result in an injury.
    // Always make sure everyone is positioned so that if you do make
    // a mistake everyone is still protected and safe!

    // As an internal safety measure, the throttle will be completely
    // turned off (value of 0.0 on a 0.0 - 1.0 scale) when the
    // pressure altitude is < 100' AGL.

    // None of the built in safety measures are sufficient for a safe
    // system!  Pressure sensor readings can glitch, bugs can creep
    // into the code over time, anything can happen.  Be extremely
    // distrustful of the propellor and always make sure your body
    // parts are never in the path of the propellor or where the
    // propellor and aircraft could go if the engine came alive
    // unexpectedly.

    // throttle

    double throttle = engine_node.getFloat("throttle");
    act_node.setFloat("throttle", throttle );

    // add in excitation signals if excitation task is running
    if ( excite_node.getBool("running") ) {
        float signal = 0.0;
        string target = "";
        int n = excite_node.getInt("channels");
        for ( int i = 0; i < n; i++ ) {
            signal = excite_node.getFloat("signal", i);
            target = excite_node.getString("target", i);
            float act_val = act_node.getFloat(target.c_str());
            act_val += signal;
            if ( target == "throttle" ) {
                if ( act_val < 0.0 ) { act_val = 0.0; }
            } else {
                if ( act_val < -1.0 ) { act_val = -1.0; }
            }
            if ( act_val > 1.0 ) { act_val = 1.0; }
            act_node.setFloat(target.c_str(), act_val);
        }
    }
    
    static bool sas_throttle_override = false;
    if ( !sas_throttle_override ) {
	if ( ap_node.getString("mode") == "sas" ) {
	    // in sas mode require a sequence of zero throttle, full
	    // throttle, and zero throttle again before throttle pass
	    // through can become active under 100' AGL

	    static int sas_throttle_state = 0;
	    if ( sas_throttle_state == 0 ) {
		if ( engine_node.getFloat("throttle") < 0.05 ) {
		    // wait for zero throttle
		    sas_throttle_state = 1;
		}
	    } else if ( sas_throttle_state == 1 ) {
		if ( engine_node.getFloat("throttle") > 0.95 ) {
		    // next wait for full throttle
		    sas_throttle_state = 2;
		}
	    } else if ( sas_throttle_state == 2 ) {
		if ( engine_node.getFloat("throttle") < 0.05 ) {
		    // next wait for zero throttle again.  Throttle pass
		    // through is now live, even under 100' AGL
		    sas_throttle_state = 3;
		    sas_throttle_override = true;
		}
	    }
	}
    }

    // for any mode that is not sas (and then only if the safety
    // override sequence has been completed), override and disable
    // throttle output if within 100' of the ground (assuming ground
    // elevation is the pressure altitude we recorded with the system
    // started up.
    if ( ! sas_throttle_override ) {
	if ( act_node.getBool("throttle_safety") ) {
	    act_node.setFloat("throttle", 0.0 );
	}
    }

    // printf("throttle = %.2f %d\n", throttle_out_node.getFloat(),
    //        servo_out.chn[2]);

    // CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!!
    // CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!!
}

PYBIND11_MODULE(actuator_mgr, m) {
    py::class_<actuators_t>(m, "actuator_mgr")
        .def(py::init<>())
        .def("init", &actuators_t::init)
        .def("update", &actuators_t::update)
    ;
}
