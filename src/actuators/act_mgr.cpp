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

#include "comms/console_link.h"
#include "comms/logging.h"
#include "include/globaldefs.h"
#include "main/globals.hxx"
#include "props/props.hxx"
#ifdef ENABLE_MNAV_SENSOR
#  include "sensors/mnav.h"
#endif
#include "util/timing.h"

#include "act_fgfs.hxx"
#include "act_mgr.h"

//
// Global variables
//

// flight control output property nodes
static SGPropertyNode *output_aileron_node = NULL;
static SGPropertyNode *output_elevator_node = NULL;
static SGPropertyNode *output_elevator_damp_node = NULL;
static SGPropertyNode *output_throttle_node = NULL;
static SGPropertyNode *output_rudder_node = NULL;

static SGPropertyNode *act_elevon_mix_node = NULL;
static SGPropertyNode *agl_alt_ft_node = NULL;

// actuator property nodes
static SGPropertyNode *act_timestamp_node = NULL;
static SGPropertyNode *act_aileron_node = NULL;
static SGPropertyNode *act_elevator_node = NULL;
static SGPropertyNode *act_throttle_node = NULL;
static SGPropertyNode *act_rudder_node = NULL;
static SGPropertyNode *act_channel5_node = NULL;
static SGPropertyNode *act_channel6_node = NULL;
static SGPropertyNode *act_channel7_node = NULL;
static SGPropertyNode *act_channel8_node = NULL;

// comm property nodes
static SGPropertyNode *act_console_skip = NULL;
static SGPropertyNode *act_logging_skip = NULL;


void Actuator_init() {
    // bind properties
    output_aileron_node = fgGetNode("/controls/flight/aileron", true);
    output_elevator_node = fgGetNode("/controls/flight/elevator", true);
    output_elevator_damp_node = fgGetNode("/controls/flight/elevator-damp", true);
    output_throttle_node = fgGetNode("/controls/engine/throttle", true);
    output_rudder_node = fgGetNode("/controls/flight/rudder", true);

    act_elevon_mix_node = fgGetNode("/config/autopilot/elevon-mixing", true);
    agl_alt_ft_node = fgGetNode("/position/altitude-agl-ft", true);

    act_timestamp_node = fgGetNode("/actuators/actuator/time-stamp", true);
    act_aileron_node = fgGetNode("/actuators/actuator/channel", 0, true);
    act_elevator_node = fgGetNode("/actuators/actuator/channel", 1, true);
    act_throttle_node = fgGetNode("/actuators/actuator/channel", 2, true);
    act_rudder_node = fgGetNode("/actuators/actuator/channel", 3, true);
    act_channel5_node = fgGetNode("/actuators/actuator/channel", 4, true);
    act_channel6_node = fgGetNode("/actuators/actuator/channel", 5, true);
    act_channel7_node = fgGetNode("/actuators/actuator/channel", 6, true);
    act_channel8_node = fgGetNode("/actuators/actuator/channel", 7, true);

    // initialize comm nodes
    act_console_skip = fgGetNode("/config/console/actuator-skip", true);
    act_logging_skip = fgGetNode("/config/logging/actuator-skip", true);

    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/actuators", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "actuator" ) {
	    string module = section->getChild("module", 0, true)->getStringValue();
	    bool enabled = section->getChild("enable", 0, true)->getBoolValue();
	    if ( !enabled ) {
		continue;
	    }
	    printf("i = %d  name = %s module = %s\n",
	    	   i, name.c_str(), module.c_str());

	    if ( module == "null" ) {
		// do nothing
	    } else if ( module == "fgfs" ) {
		fgfs_act_init( section );
#ifdef ENABLE_MNAV_SENSOR
	    } else if ( module == "mnav" ) {
		mnav_act_init();
#endif // ENABLE_MNAV_SENSOR
	    } else {
		printf("Unknown actuator = '%s' in config file\n",
		       name.c_str());
	    }
	}
    }
}


bool Actuator_update() {
    /* printf("%.2f %.2f\n", aileron_out_node->getFloatValue(),
              elevator_out_node->getFloatValue()); */
    /* static SGPropertyNode *vert_speed_fps
       = fgGetNode("/velocity/vertical-speed-fps", true); */
    /* static SGPropertyNode *true_alt
       = fgGetNode("/position/altitude-ft", true); */
    /* printf("%.1f %.2f %.2f\n",
           true_alt->getDoubleValue(),
           vert_speed_fps->getDoubleValue(),
           elevator_out_node->getFloatValue()); */

    // initialize the servo command array to central values so we don't
    // inherit junk
    //for ( int i = 0; i < 8; ++i ) {
    //    servo_out.chn[i] = 32768;
    //}

    float elevator = output_elevator_node->getFloatValue()
	+ output_elevator_damp_node->getFloatValue();

    if ( act_elevon_mix_node->getBoolValue() ) {
        // elevon mixing mode

        //aileron
        /* servo_out.chn[0] = 32768
            + (int16_t)(act_aileron_node->getFloatValue() * 32768)
            + (int16_t)(elevator * 32768); */
	act_aileron_node
	    ->setFloatValue( output_aileron_node->getFloatValue()
			      + elevator );

        //elevator
        /* servo_out.chn[1] = 32768
            + (int16_t)(act_aileron_node->getFloatValue() * 32768)
            - (int16_t)(elevator * 32768); */
	act_elevator_node
	    ->setFloatValue( output_aileron_node->getFloatValue()
			      - elevator );
    } else {
        // conventional airframe mode

        //aileron
        /* servo_out.chn[0] = 32768
	   + (int16_t)(act_aileron_node->getFloatValue() * 32768); */
	act_aileron_node
	    ->setFloatValue( output_aileron_node->getFloatValue() );

        //elevator
        /* servo_out.chn[1] = 32768
	   + (int16_t)(elevator * 32768); */
	act_elevator_node->setFloatValue( elevator );
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
    /*static int16_t last_throttle = 12000;
    int16_t target_throttle = 32768
	+ (int16_t)(act_throttle_node->getFloatValue() * 32768);
    int16_t diff = target_throttle - last_throttle;
    if ( diff > 128 ) diff = 128;
    if ( diff < -128 ) diff = -128;
    servo_out.chn[2] = last_throttle + diff;*/

    // limit throttle delta to 0.2% of full range per cycle.  At a 50hz
    // update rate it will take 10 seconds to travel the full range.
    static double last_throttle = 0.0;
    double target_throttle = output_throttle_node->getFloatValue();
    double diff = target_throttle - last_throttle;
    if ( diff > 0.002 ) { diff = 0.002; }
    if ( diff < -0.002 ) { diff = -0.002; }
    act_throttle_node->setFloatValue( last_throttle + diff );
    last_throttle = last_throttle + diff;

    // override and disable throttle output if within 100' of the
    // ground (assuming ground elevation is the pressure altitude we
    // recorded with the system started up.
    if ( agl_alt_ft_node->getDoubleValue() < 100.0 ) {
        act_throttle_node->setFloatValue( 0.0 );
	/* servo_out.chn[2] = 12000; */
    }

    // printf("throttle = %.2f %d\n", throttle_out_node->getFloatValue(),
    //        servo_out.chn[2]);

    /* last_throttle = servo_out.chn[2]; */

    // CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!!
    // CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!! CAUTION!!!

    // rudder
    /* servo_out.chn[3] = 32768
       + (int16_t)(act_rudder_node->getFloatValue() * 32768); */
    act_rudder_node->setFloatValue( output_rudder_node->getFloatValue() );

    // time stamp for logging
    /* servo_out.time = get_Time(); */
    act_timestamp_node->setDoubleValue( get_Time() );


    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/actuators", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "actuator" ) {
	    string module = section->getChild("module", 0, true)->getStringValue();
	    bool enabled = section->getChild("enable", 0, true)->getBoolValue();
	    if ( !enabled ) {
		continue;
	    }
	    if ( module == "null" ) {
		// do nothing
	    } else if ( module == "fgfs" ) {
		fgfs_act_update();
#ifdef ENABLE_MNAV_SENSOR
	    } else if ( module == "mnav" ) {
		mnav_send_short_servo_cmd( /* &servo_out */ );
#endif // ENABLE_MNAV_SENSOR
	    } else {
		printf("Unknown actuator = '%s' in config file\n",
		       name.c_str());
	    }
	}
    }

    if ( console_link_on || log_to_file ) {
	uint8_t buf[256];
	int size = packetizer->packetize_actuator( buf );

	if ( console_link_on ) {
	    console_link_actuator( buf, size, act_console_skip->getIntValue() );
	}

	if ( log_to_file ) {
	    log_actuator( buf, size, act_logging_skip->getIntValue() );
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
	if ( name == "actuator" ) {
	    string module = section->getChild("module", 0, true)->getStringValue();
	    bool enabled = section->getChild("enable", 0, true)->getBoolValue();
	    if ( !enabled ) {
		continue;
	    }
	    if ( module == "null" ) {
		// do nothing
	    } else if ( module == "fgfs" ) {
		fgfs_act_close();
#ifdef ENABLE_MNAV_SENSOR
	    } else if ( module == "mnav" ) {
		// do nothing
#endif // ENABLE_MNAV_SENSOR
	    } else {
		printf("Unknown actuator = '%s' in config file\n",
		       name.c_str());
	    }
	}
    }
}
