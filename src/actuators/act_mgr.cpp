/**
 * \file: act_mgr.cpp
 *
 * Front end management interface for output actuators
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@gmail.com
 *
 * $Id: act_mgr.cpp,v 1.2 2009/04/30 14:39:38 curt Exp $
 */


#include <math.h>
#include <string.h>

#include "globaldefs.h"

#include "comms/logging.h"
#include "props/props.hxx"
#include "sensors/mnav.h"
#include "util/timing.h"

#include "act_mgr.h"

//
// Global variables
//

static actuator_output_t output = actNone;
struct servo servo_out;

// actuator property nodes
static SGPropertyNode *actuator_output_node = NULL;
static SGPropertyNode *agl_alt_ft_node = NULL;
static SGPropertyNode *aileron_out_node = NULL;
static SGPropertyNode *elevator_out_node = NULL;
static SGPropertyNode *elevator_damp_node = NULL;
static SGPropertyNode *throttle_out_node = NULL;
static SGPropertyNode *rudder_out_node = NULL;
static SGPropertyNode *elevon_mix = NULL;


void Actuator_init() {
    // initialize actuator property nodes
    actuator_output_node = fgGetNode("/config/actuators/output", true);
    if ( strcmp(actuator_output_node->getStringValue(), "mnav") == 0 ) {
	output = actMNAV;
    }

    agl_alt_ft_node = fgGetNode("/position/altitude-agl-ft", true);
    aileron_out_node = fgGetNode("/controls/flight/aileron", true);
    elevator_out_node = fgGetNode("/controls/flight/elevator", true);
    elevator_damp_node = fgGetNode("/controls/flight/elevator-damp", true);
    throttle_out_node = fgGetNode("/controls/engine/throttle", true);
    rudder_out_node = fgGetNode("/controls/flight/rudder", true);
    elevon_mix = fgGetNode("/config/autopilot/elevon-mixing", true);

    switch ( output ) {

    case actMNAV:
	// nothing to do
	break;

    case actNone:
	// nothing to do
	break;

    default:
	if ( display_on ) {
	    printf("Warning: (init) no actuator output defined\n");
	}
    }

}


bool Actuator_update() {
    bool fresh_data = false;

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


    switch ( output ) {

    case actMNAV:
	// send commanded servo positions to the MNAV
	mnav_send_short_servo_cmd( &servo_out );
	break;

    case actNone:
	// noop
	break;

    default:
	if ( display_on ) {
	    printf("Warning: (update) no actuator source defined\n");
	}
    }

    if ( fresh_data ) {
	// publish values to property tree

	// if ( console_link_on ) {
	//     console_link_imu( &imupacket );
	// }

	// if ( log_to_file ) {
	//     log_imu( &imupacket );
	// }
    }

    return fresh_data;
}


void Actuators_close() {
    switch ( output ) {

    case actMNAV:
	// nop
	break;

    default:
	if ( display_on ) {
	    printf("Warning: (close) no pressure source defined\n");
	}
    }
}
