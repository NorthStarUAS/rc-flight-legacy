/**
 * \file: act_mgr.cpp
 *
 * Front end management interface for output actuators
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@gmail.com
 *
 * $Id: act_mgr.cpp,v 1.3 2009/08/25 15:04:01 curt Exp $
 */

#include <cstdio>

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "include/ugear_config.h"

#include "comms/logging.h"
#include "include/globaldefs.h"
#include "props/props.hxx"
#ifdef ENABLE_MNAV_SENSOR
#  include "sensors/mnav.h"
#endif
#include "util/timing.h"

#include "act_mgr.h"

//
// Global variables
//

struct servo servo_out;

// actuator property nodes
static SGPropertyNode *agl_alt_ft_node = NULL;
static SGPropertyNode *aileron_out_node = NULL;
static SGPropertyNode *elevator_out_node = NULL;
static SGPropertyNode *elevator_damp_node = NULL;
static SGPropertyNode *throttle_out_node = NULL;
static SGPropertyNode *rudder_out_node = NULL;
static SGPropertyNode *elevon_mix = NULL;


void Actuator_init() {
    // bind properties
    agl_alt_ft_node = fgGetNode("/position/altitude-agl-ft", true);
    aileron_out_node = fgGetNode("/controls/flight/aileron", true);
    elevator_out_node = fgGetNode("/controls/flight/elevator", true);
    elevator_damp_node = fgGetNode("/controls/flight/elevator-damp", true);
    throttle_out_node = fgGetNode("/controls/engine/throttle", true);
    rudder_out_node = fgGetNode("/controls/flight/rudder", true);
    elevon_mix = fgGetNode("/config/autopilot/elevon-mixing", true);

    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/actuators", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "null" ) {
	    // do nothing
#ifdef ENABLE_MNAV_SENSOR
	} else if ( name == "mnav" ) {
	    // do nothing
#endif // ENABLE_MNAV_SENSOR
	} else {
	    printf("Unknown actuator = '%s' in config file\n",
		   name.c_str());
	}
    }
}


bool Actuator_update() {
    /* printf("%.2f %.2f\n", aileron_out_node->getDoubleValue(),
              elevator_out_node->getDoubleValue()); */
    /* static SGPropertyNode *vert_speed_fps
       = fgGetNode("/velocities/vertical-speed-fps", true); */
    /* static SGPropertyNode *true_alt
       = fgGetNode("/position/altitude-ft", true); */
    /* printf("%.1f %.2f %.2f\n",
           true_alt->getDoubleValue(),
           vert_speed_fps->getDoubleValue(),
           elevator_out_node->getDoubleValue()); */

    // initialize the servo command array to central values so we don't
    // inherit junk
    for ( int i = 0; i < 8; ++i ) {
        servo_out.chn[i] = 32768;
    }

    float elevator = elevator_out_node->getDoubleValue()
	+ elevator_damp_node->getDoubleValue();

    if ( elevon_mix->getBoolValue() ) {
        // elevon mixing mode

        //aileron
        servo_out.chn[0] = 32768
            + (int16_t)(aileron_out_node->getDoubleValue() * 32768)
            + (int16_t)(elevator * 32768);

        //elevator
        servo_out.chn[1] = 32768
            + (int16_t)(aileron_out_node->getDoubleValue() * 32768)
            - (int16_t)(elevator * 32768);
    } else {
        // conventional airframe mode

        //aileron
        servo_out.chn[0] = 32768
            + (int16_t)(aileron_out_node->getDoubleValue() * 32768);

        //elevator
        servo_out.chn[1] = 32768
            + (int16_t)(elevator * 32768);
    }

    // CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!!
    // CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!!

    // Placing the engine throttle under autopilot control requires
    // EXTREME care!!!!

    // Propellers are constructed of sharpened knife material.
    // Electric motors don't quit and give up if they encounter intial
    // resistance.  Severe injuries to hand or face or any other body
    // part in the vicinity of the motor or prop can occur at any
    // time.

    // Care must be taken during initial setup, and then from that
    // point on during all operational, testing, and ground handling
    // phases.  Extreme vigilance must always be maintianed if the
    // autopilot has control of the throttle.

    // I cannot stress this point enough!!!  One nanosecond of
    // distraction or loss of focus can result in severe lifelong
    // injury or death!  Do not take your fingers or face for granted.
    // Always maintain utmost caution and correct safety procedures to
    // ensure safe operation with a throttle enabled UAS.

    // As an internal safety measure, the throttle will be completely
    // turned off (value of 12000 on a 0-65535 scale) when the
    // pressure altitude is < 50m AGL.

    // None of the built in safety measures are sufficient for a safe
    // system!  Pressure sensor readings can glitch, bugs can creep
    // into the code over time, anything can happen.  Be extremely
    // distrustful of the propellor and always make sure your body
    // parts are never in the path of the propellor or where the
    // propellor and aircraft could go if the engine came alive
    // unexpectedly.

    // throttle

    // limit throttle change to 128 units per cycle ... which means it
    // takes about 10 seconds to traverse a range of 32768 (about full
    // range) assuming 25 cycles per second.
    static int16_t last_throttle = 12000;
    int16_t target_throttle = 32768
	+ (int16_t)(throttle_out_node->getDoubleValue() * 32768);
    int16_t diff = target_throttle - last_throttle;
    if ( diff > 128 ) diff = 128;
    if ( diff < -128 ) diff = -128;
    servo_out.chn[2] = last_throttle + diff;

    // override and disable throttle output if within 100' of the
    // ground (assuming ground elevation is the pressure altitude we
    // recorded with the system started up.
    if ( agl_alt_ft_node->getDoubleValue() < 100.0 ) {
        servo_out.chn[2] = 12000;
    }

    // printf("throttle = %.2f %d\n", throttle_out_node->getDoubleValue(),
    //        servo_out.chn[2]);

    last_throttle = servo_out.chn[2];

    // CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!!
    // CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!!

    // rudder
    servo_out.chn[3] = 32768
        + (int16_t)(rudder_out_node->getDoubleValue() * 32768);

    // time stamp the packet for logging
    servo_out.time = get_Time();


    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/actuators", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "null" ) {
	    // do nothing
#ifdef ENABLE_MNAV_SENSOR
	} else if ( name == "mnav" ) {
	    mnav_send_short_servo_cmd( &servo_out );
#endif // ENABLE_MNAV_SENSOR
	} else {
	    printf("Unknown actuator = '%s' in config file\n",
		   name.c_str());
	}
    }

    return true;
}


void Actuators_close() {
    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/actuators", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "null" ) {
	    // do nothing
#ifdef ENABLE_MNAV_SENSOR
	} else if ( name == "mnav" ) {
	    // noop
#endif // ENABLE_MNAV_SENSOR
	} else {
	    printf("Unknown actuator = '%s' in config file\n",
		   name.c_str());
	}
    }
}
