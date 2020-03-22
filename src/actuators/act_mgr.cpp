/**
 * \file: act_mgr.cpp
 *
 * Front end management interface for output actuators
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@flightgear.org
 *
 * $Id: act_mgr.cpp,v 1.3 2009/08/25 15:04:01 curt Exp $
 */

#include <pyprops.h>

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include <sstream>
#include <string>
#include <vector>
using std::ostringstream;
using std::string;
using std::vector;

#include "control/control.h"
#include "include/globaldefs.h"
#include "init/globals.h"
#include "util/timing.h"

#include "drivers/APM2.h"
#include "drivers/Aura3/Aura3.h"

#include "act_mgr.h"

//
// Global variables
//

// property nodes
static pyPropertyNode flight_node;
static pyPropertyNode engine_node;
static pyPropertyNode pilot_node;
static pyPropertyNode act_node;
static pyPropertyNode ap_node;
static pyPropertyNode excite_node;
static vector<pyPropertyNode> sections;

void Actuator_init() {
    // bind properties
    flight_node = pyGetNode("/controls/flight", true);
    engine_node = pyGetNode("/controls/engine", true);
    excite_node = pyGetNode("/task/excite", true);
    pilot_node = pyGetNode("/sensors/pilot_input", true);
    act_node = pyGetNode("/actuators", true);
    ap_node = pyGetNode("/autopilot", true);
    
    // traverse configured modules
    pyPropertyNode group_node = pyGetNode("/config/actuators", true);
    vector<string> children = group_node.getChildren();
    printf("Found %d actuator sections\n", (int)children.size());
    for ( unsigned int i = 0; i < children.size(); i++ ) {
	pyPropertyNode section = group_node.getChild(children[i].c_str());
	sections.push_back(section);
	string module = section.getString("module");
	bool enabled = section.getBool("enable");
	if ( !enabled ) {
	    continue;
	}
	printf("actuator: %d = %s\n", i, module.c_str());
	if ( module == "null" ) {
	    // do nothing
	} else if ( module == "APM2" ) {
	    APM2_act_init( &section );
	    // don't go anywhere until the acuator is configured.
	    // this will also force the APM2 into binary mode as soon
	    // as it starts seeing our binary config packets coming
	    // in.
	    APM2_act_update();
	    while ( ! APM2_actuator_configured ) {
		usleep(250000);
		APM2_act_update();
	    }
	} else if ( module == "Aura3" ) {
	    Aura3_act_init( &section );
	    // don't go anywhere until the acuator is configured.
	    // this will also force the Aura3 into binary mode as soon
	    // as it starts seeing our binary config packets coming
	    // in.
	    Aura3_act_update();
	    while ( ! Aura3_actuator_configured ) {
		usleep(250000);
		Aura3_act_update();
	    }
	} else {
	    printf("Unknown actuator = '%s' in config file\n",
		   module.c_str());
	}
    }
}


static void set_actuator_values() {
    float aileron = flight_node.getDouble("aileron");
    act_node.setDouble( "aileron", aileron );

    float elevator = flight_node.getDouble("elevator");
    act_node.setDouble( "elevator", elevator );

    // rudder
    float rudder = flight_node.getDouble("rudder");
    act_node.setDouble( "rudder", rudder );

    double flaps = flight_node.getDouble("flaps");
    act_node.setDouble("flaps", flaps );

    double gear = flight_node.getDouble("gear");
    act_node.setDouble("gear", gear );

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

    double throttle = engine_node.getDouble("throttle");
    act_node.setDouble("throttle", throttle );

    // add in excitation signals if excitation task is running
    if ( excite_node.getBool("running") ) {
        float signal = 0.0;
        string target = "";
        int n = excite_node.getLong("channels");
        for ( int i = 0; i < n; i++ ) {
            signal = excite_node.getDouble("signal", i);
            target = excite_node.getString("target", i);
            float act_val = act_node.getDouble(target.c_str());
            act_val += signal;
            if ( target == "throttle" ) {
                if ( act_val < 0.0 ) { act_val = 0.0; }
            } else {
                if ( act_val < -1.0 ) { act_val = -1.0; }
            }
            if ( act_val > 1.0 ) { act_val = 1.0; }
            act_node.setDouble(target.c_str(), act_val);
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
		if ( engine_node.getDouble("throttle") < 0.05 ) {
		    // wait for zero throttle
		    sas_throttle_state = 1;
		}
	    } else if ( sas_throttle_state == 1 ) {
		if ( engine_node.getDouble("throttle") > 0.95 ) {
		    // next wait for full throttle
		    sas_throttle_state = 2;
		}
	    } else if ( sas_throttle_state == 2 ) {
		if ( engine_node.getDouble("throttle") < 0.05 ) {
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
	    act_node.setDouble("throttle", 0.0 );
	}
    }

    // printf("throttle = %.2f %d\n", throttle_out_node.getDouble(),
    //        servo_out.chn[2]);

    // CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!!
    // CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!!
}


bool Actuator_update() {
    // printf("Actuator_update()\n");

    // time stamp for logging
    act_node.setDouble( "timestamp", get_Time() );
    set_actuator_values();
    
    // traverse configured modules
    for ( unsigned int i = 0; i < sections.size(); i++ ) {
	string module = sections[i].getString("module");
	bool enabled = sections[i].getBool("enable");
	if ( !enabled ) {
	    continue;
	}
	if ( module == "null" ) {
	    // do nothing
	} else if ( module == "APM2" ) {
	    APM2_act_update();
	} else if ( module == "Aura3" ) {
	    Aura3_act_update();
	} else {
	    printf("Unknown actuator = '%s' in config file\n",
		   module.c_str());
	}
    }
    
    return true;
}


void Actuators_close() {
    // traverse configured modules
    for ( unsigned int i = 0; i < sections.size(); i++ ) {
	string module = sections[i].getString("module");
	bool enabled = sections[i].getBool("enable");
	if ( !enabled ) {
	    continue;
	}
	if ( module == "null" ) {
	    // do nothing
	} else if ( module == "APM2" ) {
	    APM2_act_close();
	} else if ( module == "Aura3" ) {
	    Aura3_act_close();
	} else {
	    printf("Unknown actuator = '%s' in config file\n",
		   module.c_str());
	}
    }
}
