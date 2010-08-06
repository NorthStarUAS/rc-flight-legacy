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
static SGPropertyNode *ap_heading_mode_node = NULL;
static SGPropertyNode *ap_yaw_mode_node = NULL;
static SGPropertyNode *ap_altitude_mode_node = NULL;
static SGPropertyNode *ap_speed_mode_node = NULL;

static SGPropertyNode *heading_lock_node = NULL;
static SGPropertyNode *yaw_lock_node = NULL;
static SGPropertyNode *altitude_lock_node = NULL;
static SGPropertyNode *speed_lock_node = NULL;

// temporary ... set target speed to current speed when autopilot is
// activated
static SGPropertyNode *cur_speed_node = NULL;
static SGPropertyNode *initial_speed_node = NULL;
static SGPropertyNode *target_speed_node = NULL;


static void bind_properties() {
    ap_master_switch_node = fgGetNode("/autopilot/master-switch", true);
    ap_heading_mode_node = fgGetNode("/autopilot/heading-mode", true);
    ap_yaw_mode_node = fgGetNode("/autopilot/yaw-mode", true);
    ap_altitude_mode_node = fgGetNode("/autopilot/altitude-mode", true);
    ap_speed_mode_node = fgGetNode("/autopilot/speed-mode", true);

    heading_lock_node = fgGetNode("/autopilot/locks/heading", true);
    yaw_lock_node = fgGetNode("/autopilot/locks/yaw", true);
    altitude_lock_node = fgGetNode("/autopilot/locks/altitude", true);
    speed_lock_node = fgGetNode("/autopilot/locks/speed", true);

    cur_speed_node = fgGetNode("/velocity/airspeed-kt", true);
    initial_speed_node = fgGetNode("/config/autopilot/initial-speed-kt", true);
    target_speed_node = fgGetNode("/autopilot/settings/target-speed-kt", true);
}


void control_init() {
    // initialize the autopilot class and build the structures from the
    // configuration file values

    bind_properties();

    ap.init();
    ap.build();

    // set default autopilot modes
    ap_heading_mode_node->setStringValue("route");
    ap_yaw_mode_node->setStringValue("turn-coord");
    ap_altitude_mode_node->setStringValue("pitch");
    ap_speed_mode_node->setStringValue("throttle");

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
    static bool autopilot_enabled = false;

    if ( ap_master_switch_node->getBoolValue() ) {
	if ( !autopilot_enabled ) {
	    // autopilot is just activated, set lock modes
	    heading_lock_node
		->setStringValue( ap_heading_mode_node->getStringValue() );
	    yaw_lock_node
		->setStringValue( ap_yaw_mode_node->getStringValue() );
	    altitude_lock_node
		->setStringValue( ap_altitude_mode_node->getStringValue() );
	    speed_lock_node
		->setStringValue( ap_speed_mode_node->getStringValue() );

	    float target_speed_kt = target_speed_node->getFloatValue();
	    float initial_speed_kt = initial_speed_node->getFloatValue();
	    if ( target_speed_kt < 0.1 ) {
		target_speed_node->setFloatValue( initial_speed_kt );
	    }

	    autopilot_enabled = true;
	}
    } else {
	if ( autopilot_enabled ) {
	    // autopilot is just de-activated, clear lock modes
	    heading_lock_node->setStringValue( "" );
	    yaw_lock_node->setStringValue( "" );
	    altitude_lock_node->setStringValue( "" );
	    speed_lock_node->setStringValue( "" );
	    autopilot_enabled = false;
	}
    }

    if ( autopilot_enabled ) {
	// update the autopilot stages
	ap.update( dt );
    }
}


void control_close() {
  // nothing to see here, move along ...
}
