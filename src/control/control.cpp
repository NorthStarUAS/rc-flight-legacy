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
#include "comms/uplink.h"
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


void control_init() {
    // initialize the autopilot class and build the structures from the
    // configuration file values
    ap.init();
    ap.build();
}


void control_reset() {
    // initialization:
    if ( display_on ) {
	printf("Initializing autopilot\n");
    }
}


void control_update(short flight_mode)
{
    // make a quick exit if we are disabled
    if ( !autopilot_active ) {
      return;
    }

    // reset the autopilot if requested
    if ( autopilot_reinit ) {
      control_reset();
      autopilot_reinit = false;
    }

    // optional: use channel #6 to change the autopilot target value
    // double min_value = -35.0;
    // double max_value = 35.0;
    // double tgt_value = (max_value - min_value) *
    //   ((double)servo_in.chn[5] / 65535.0) + min_value;

    // update the autopilot stages
    ap.update( 0.04 );	// dt = 1/25


}


void control_close() {
  // nothing to see here, move along ...
}
