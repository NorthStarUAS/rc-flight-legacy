/******************************************************************************
 * FILE: control.cpp
 * DESCRIPTION: high level control/autopilot interface
 *
 ******************************************************************************/

#include <stdio.h>
#include <sys/time.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>

#include "comms/logging.h"
#include "include/globaldefs.h"
#include "sensors/mnav.h"

#include "util.h"
#include "xmlauto.hxx"

#include "control.h"


//
// global variables
//

// the "FlightGear" autopilot
static FGXMLAutopilot ap;


// autopilot control properties
static SGPropertyNode *ap_master_switch_node = NULL;
static SGPropertyNode *fcs_mode_node = NULL;
//static SGPropertyNode *ap_heading_mode_node = NULL;
//static SGPropertyNode *ap_roll_mode_node = NULL;
//static SGPropertyNode *ap_yaw_mode_node = NULL;
//static SGPropertyNode *ap_altitude_mode_node = NULL;
//static SGPropertyNode *ap_speed_mode_node = NULL;
//static SGPropertyNode *ap_pitch_mode_node = NULL;

static SGPropertyNode *heading_lock_node = NULL;
static SGPropertyNode *roll_lock_node = NULL;
static SGPropertyNode *yaw_lock_node = NULL;
static SGPropertyNode *altitude_lock_node = NULL;
static SGPropertyNode *speed_lock_node = NULL;
static SGPropertyNode *pitch_lock_node = NULL;

static SGPropertyNode *roll_deg_node = NULL;
static SGPropertyNode *pitch_deg_node = NULL;
static SGPropertyNode *cur_speed_node = NULL;
static SGPropertyNode *initial_speed_node = NULL;
static SGPropertyNode *target_roll_deg_node = NULL;
static SGPropertyNode *target_pitch_deg_node = NULL;
static SGPropertyNode *target_speed_node = NULL;


static void bind_properties() {
    ap_master_switch_node = fgGetNode("/autopilot/master-switch", true);
    fcs_mode_node = fgGetNode("/config/autopilot/fcs-mode", true);
    // ap_heading_mode_node = fgGetNode("/autopilot/heading-mode", true);
    // ap_roll_mode_node = fgGetNode("/autopilot/roll-mode", true);
    // ap_yaw_mode_node = fgGetNode("/autopilot/yaw-mode", true);
    // ap_altitude_mode_node = fgGetNode("/autopilot/altitude-mode", true);
    // ap_speed_mode_node = fgGetNode("/autopilot/speed-mode", true);
    // ap_pitch_mode_node = fgGetNode("/autopilot/pitch-mode", true);

    heading_lock_node = fgGetNode("/autopilot/locks/heading", true);
    roll_lock_node = fgGetNode("/autopilot/locks/roll", true);
    yaw_lock_node = fgGetNode("/autopilot/locks/yaw", true);
    altitude_lock_node = fgGetNode("/autopilot/locks/altitude", true);
    speed_lock_node = fgGetNode("/autopilot/locks/speed", true);
    pitch_lock_node = fgGetNode("/autopilot/locks/pitch", true);

    roll_deg_node = fgGetNode("/orientation/roll-deg", true);
    pitch_deg_node = fgGetNode("/orientation/pitch-deg", true);
    cur_speed_node = fgGetNode("/velocity/airspeed-kt", true);
    initial_speed_node = fgGetNode("/config/autopilot/initial-speed-kt", true);
    target_roll_deg_node
	= fgGetNode("/autopilot/settings/target-roll-deg", true);
    target_pitch_deg_node
	= fgGetNode("/autopilot/settings/target-pitch-deg", true);
    target_speed_node = fgGetNode("/autopilot/settings/target-speed-kt", true);
}


void control_init() {
    // initialize the autopilot class and build the structures from the
    // configuration file values

    bind_properties();

    ap.init();
    ap.build();

    // set default autopilot modes
    // ap_heading_mode_node->setStringValue("route");
    // ap_roll_mode_node->setStringValue("aileron");
    // ap_yaw_mode_node->setStringValue("turn-coord");
    // ap_altitude_mode_node->setStringValue("pitch");
    // ap_speed_mode_node->setStringValue("throttle");
    // ap_pitch_mode_node->setStringValue("elevator");

    if ( display_on ) {
	printf("Autopilot initialized\n");
    }
}


void control_reinit() {
    // reread autopilot configuration from the property tree and reset
    // all stages (i.e. real time gain tuning)

    ap.reinit();
}


void control_update(double dt)
{
    static bool fcs_enabled = false;

    if ( ap_master_switch_node->getBoolValue() ) {
	if ( !fcs_enabled ) {
	    if ( strcmp(fcs_mode_node->getStringValue(), "route") == 0 ) {
		// fcs is just activated, set lock modes for "route"
		heading_lock_node->setStringValue( "route" );
		roll_lock_node->setStringValue( "aileron" );
		yaw_lock_node->setStringValue( "turn-coord" );
		altitude_lock_node->setStringValue( "pitch" );
		speed_lock_node->setStringValue( "throttle" );
		pitch_lock_node->setStringValue( "elevator" );

		float target_speed_kt = target_speed_node->getFloatValue();
		float initial_speed_kt = initial_speed_node->getFloatValue();
		if ( target_speed_kt < 0.1 ) {
		    target_speed_node->setFloatValue( initial_speed_kt );
		}
		fcs_enabled = true;
	    } else if ( strcmp(fcs_mode_node->getStringValue(), "sas") == 0 ) {
		// fcs is just activated, set lock modes for "sas"
		heading_lock_node->setStringValue( "" );
		roll_lock_node->setStringValue( "aileron" );
		yaw_lock_node->setStringValue( "" );
		altitude_lock_node->setStringValue( "" );
		speed_lock_node->setStringValue( "" );
		pitch_lock_node->setStringValue( "elevator" );

		float target_roll_deg = roll_deg_node->getFloatValue();
		if ( target_roll_deg > 45.0 ) { target_roll_deg = 45.0; }
		if ( target_roll_deg < -45.0 ) { target_roll_deg = -45.0; }
		target_roll_deg_node->setFloatValue( target_roll_deg );

		float target_pitch_deg = pitch_deg_node->getFloatValue();
		if ( target_pitch_deg > 15.0 ) { target_pitch_deg = 15.0; }
		if ( target_pitch_deg < -15.0 ) { target_pitch_deg = -15.0; }
		target_pitch_deg_node->setFloatValue( target_pitch_deg );

		float target_speed_kt = target_speed_node->getFloatValue();
		float initial_speed_kt = initial_speed_node->getFloatValue();
		if ( target_speed_kt < 0.1 ) {
		    target_speed_node->setFloatValue( initial_speed_kt );
		}
		fcs_enabled = true;
	    }
	}
    } else {
	if ( fcs_enabled ) {
	    // autopilot is just de-activated, clear lock modes
	    heading_lock_node->setStringValue( "" );
	    roll_lock_node->setStringValue( "" );
	    yaw_lock_node->setStringValue( "" );
	    altitude_lock_node->setStringValue( "" );
	    speed_lock_node->setStringValue( "" );
	    pitch_lock_node->setStringValue( "" );
	    fcs_enabled = false;
	}
    }

    if ( fcs_enabled ) {
	// update the autopilot stages
	ap.update( dt );
    }
}


void control_close() {
  // nothing to see here, move along ...
}
