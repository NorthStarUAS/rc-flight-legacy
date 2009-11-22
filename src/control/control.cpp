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


void control_init() {
    // initialize the autopilot class and build the structures from the
    // configuration file values

    ap.init();
    ap.build();

    if ( display_on ) {
	printf("Initializing autopilot\n");
    }
}


void control_reset() {
    // initialization
}


void control_update(short flight_mode)
{
#if 0
    // FIXME: we need a more generic autopilot mode switching system.
    // I can envision at least 3 modes: (1) Manual pass through, (2)
    // Fly by wire, and (3) full autopilot

    // make a quick exit if we are disabled
    if ( !autopilot_active ) {
      return;
    }

    // reset the autopilot if requested
    if ( autopilot_reinit ) {
      control_reset();
      autopilot_reinit = false;
    }
#endif

    // update the autopilot stages
    ap.update( 0.04 );	// dt = 1/25
}


void control_close() {
  // nothing to see here, move along ...
}
