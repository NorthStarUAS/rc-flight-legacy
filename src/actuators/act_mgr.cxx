/**
 * \file: act_mgr.cpp
 *
 * Front end management interface for output actuators
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@flightgear.org
 *
 * $Id: act_mgr.cpp,v 1.3 2009/08/25 15:04:01 curt Exp $
 */

#include "python/pyprops.hxx"

#include <math.h>
#include <string.h>
#include <stdio.h>

#include <string>
#include <vector>
using std::string;
using std::vector;

#include "comms/remote_link.hxx"
#include "comms/logging.hxx"
#include "include/globaldefs.h"
#include "init/globals.hxx"
#include "util/myprof.h"
#include "util/timing.h"

#include "act_fgfs.hxx"
#include "act_goldy2.hxx"
#include "sensors/APM2.hxx"

#include "act_mgr.hxx"

//
// Global variables
//

// property nodes
static pyPropertyNode flight_node;
static pyPropertyNode engine_node;
static pyPropertyNode acts_node;
static pyPropertyNode act_node;
static pyPropertyNode limits_node;
static pyPropertyNode fcs_node;
static pyPropertyNode ap_node;
static pyPropertyNode remote_link_node;
static pyPropertyNode logging_node;

static myprofile debug6a;
static myprofile debug6b;


void Actuator_init() {
    debug6a.set_name("debug6a act update and output");
    debug6b.set_name("debug6b act console logging");

    // bind properties
    flight_node = pyGetNode("/controls/flight", true);
    engine_node = pyGetNode("/controls/engine", true);
    acts_node = pyGetNode("/actuators", true);
    act_node = pyGetNode("/actuators/actuator", true);
#define NUM_ACTUATORS 8
    act_node.setLen("channel", NUM_ACTUATORS-1, 0.0);
    limits_node = pyGetNode("/config/actuators/limits", true);
    fcs_node = pyGetNode("/config/fcs", true);
    ap_node = pyGetNode("/autopilot", true);
    remote_link_node = pyGetNode("/config/remote_link", true);
    logging_node = pyGetNode("/config/logging", true);

    // traverse configured modules
    pyPropertyNode group_node = pyGetNode("/config/actuators", true);
    vector<string> children = group_node.getChildren();
    printf("Found %d actuator sections\n", children.size());
    for ( unsigned int i = 0; i < children.size(); i++ ) {
	pyPropertyNode section = group_node.getChild(children[i].c_str());
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
	} else if ( module == "fgfs" ) {
	    fgfs_act_init( &section );
	} else if ( module == "Goldy2" ) {
	    goldy2_act_init( &section );
	} else {
	    printf("Unknown actuator = '%s' in config file\n",
		   module.c_str());
	}
    }
}


static void set_actuator_values_ap() {
    float aileron = flight_node.getDouble("aileron");
    if ( aileron < limits_node.getDouble("aileron_min") ) {
	aileron = limits_node.getDouble("aileron_min");
    }
    if ( aileron > limits_node.getDouble("aileron_max") ) {
	aileron = limits_node.getDouble("aileron_max");
    }
    act_node.setDouble( "channel", 0, aileron );

    float elevator = flight_node.getDouble("elevator");
    if ( elevator < limits_node.getDouble("elevator_min") ) {
	elevator = limits_node.getDouble("elevator_min");
    }
    if ( elevator > limits_node.getDouble("elevator_max") ) {
	elevator = limits_node.getDouble("elevator_max");
    }
    act_node.setDouble( "channel", 1, elevator );

    // rudder
    float rudder = flight_node.getDouble("rudder");
    if ( rudder < limits_node.getDouble("rudder_min") ) {
	rudder = limits_node.getDouble("rudder_min");
    }
    if ( rudder > limits_node.getDouble("rudder_max") ) {
	rudder = limits_node.getDouble("rudder_max");
    }
    act_node.setDouble( "channel", 3, rudder );

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
    if ( throttle < limits_node.getDouble("throttle_min") ) {
	throttle = limits_node.getDouble("throttle_min");
    }
    if ( throttle > limits_node.getDouble("throttle_max") ) {
	throttle = limits_node.getDouble("throttle_max");
    }
    act_node.setDouble("channel", 2, throttle );

    static bool sas_throttle_override = false;

    if ( !sas_throttle_override ) {
	if ( fcs_node.getString("mode") == "sas" ) {
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
	if ( acts_node.getBool("throttle_safety") ) {
	    act_node.setDouble("channel", 2, 0.0 );
	}
    }

    // printf("throttle = %.2f %d\n", throttle_out_node.getDouble(),
    //        servo_out.chn[2]);

    // CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!!
    // CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!!
}


static void set_actuator_values_pilot() {
    // The following lines would act as a manual pass-through at the
    // ugear level.  However, manaul pass-through is handled more
    // efficiently (less latency) directly on APM2.x hardware.
    //
    // act_aileron_node.setDouble( pilot_aileron_node.getDouble() );
    // act_elevator_node.setDouble( pilot_elevator_node.getDouble() );
    // act_throttle_node.setDouble( pilot_throttle_node.getDouble() );
    // act_rudder_node.setDouble( pilot_rudder_node.getDouble() );
}


bool Actuator_update() {
    debug6a.start();

    // time stamp for logging
    act_node.setDouble( "timestamp", get_Time() );
    if ( ap_node.getBool("master_switch") ) {
	set_actuator_values_ap();
    } else {
	set_actuator_values_pilot();
    }

    printf("begin actuator_update()\n");
    
    // traverse configured modules
    pyPropertyNode group_node = pyGetNode("/config/actuators", true);
    vector<string> children = group_node.getChildren();
    for ( unsigned int i = 0; i < children.size(); i++ ) {
	pyPropertyNode section = group_node.getChild(children[i].c_str());
	string module = section.getString("module");
	bool enabled = section.getBool("enable");
	if ( !enabled ) {
	    continue;
	}
	if ( module == "null" ) {
	    // do nothing
	} else if ( module == "APM2" ) {
	    APM2_act_update();
	} else if ( module == "fgfs" ) {
	    fgfs_act_update();
	} else if ( module == "Goldy2" ) {
	    goldy2_act_update();
	} else {
	    printf("Unknown actuator = '%s' in config file\n",
		   module.c_str());
	}
    }

    printf("end actuator_update()\n");

    debug6a.stop();

    debug6b.start();

    if ( remote_link_on || log_to_file ) {
	// actuators

	uint8_t buf[256];
	int size = packetizer->packetize_actuator( buf );

	if ( remote_link_on ) {
	    remote_link_actuator( buf, size, remote_link_node.getLong("actuator_skip") );
	}

	if ( log_to_file ) {
	    log_actuator( buf, size, logging_node.getLong("actuator_skip") );
	}
    }

    debug6b.stop();

    return true;
}


void Actuators_close() {
    // traverse configured modules
    pyPropertyNode group_node = pyGetNode("/config/actuators", true);
    vector<string> children = group_node.getChildren();
    for ( unsigned int i = 0; i < children.size(); i++ ) {
	pyPropertyNode section = group_node.getChild(children[i].c_str());
	string module = section.getString("module");
	bool enabled = section.getBool("enable");
	if ( !enabled ) {
	    continue;
	}
	if ( module == "null" ) {
	    // do nothing
	} else if ( module == "APM2" ) {
	    APM2_act_close();
	} else if ( module == "fgfs" ) {
	    fgfs_act_close();
	} else if ( module == "Goldy2" ) {
	    goldy2_act_close();
	} else {
	    printf("Unknown actuator = '%s' in config file\n",
		   module.c_str());
	}
    }
}
