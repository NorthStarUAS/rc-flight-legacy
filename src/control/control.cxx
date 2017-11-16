// control.cpp - high level control/autopilot interface
//
// Written by Curtis Olson, started January 2006.
//
// Copyright (C) 2006  Curtis L. Olson  - http://www.flightgear.org/~curt
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Library General Public
// License as published by the Free Software Foundation; either
// version 2 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Library General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//

#include "python/pyprops.hxx"

#include <stdio.h>
#include <sys/time.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

#include "comms/display.hxx"
#include "comms/logging.hxx"
#include "comms/remote_link.hxx"
#include "include/globaldefs.h"
#include "init/globals.hxx"
#include "python/pymodule.hxx"

#include "include/util.h"
#include "ap.hxx"
#include "tecs.hxx"

#include "control.hxx"


// global variables
static pyModuleBase navigation;
static AuraAutopilot ap;


// property nodes
static pyPropertyNode ap_node;
static pyPropertyNode remote_link_node;
static pyPropertyNode logging_node;
static pyPropertyNode comms_node;
static pyPropertyNode pilot_node;
static pyPropertyNode flight_node;
static pyPropertyNode engine_node;

static int remote_link_skip = 0;
static int logging_skip = 0;

static void bind_properties() {
    ap_node = pyGetNode( "/autopilot", true );
    remote_link_node = pyGetNode( "/config/remote_link", true );
    logging_node = pyGetNode( "/config/logging", true );
    comms_node = pyGetNode( "/comms/remote_link", true);
    pilot_node = pyGetNode("/sensors/pilot_input", true);
    flight_node = pyGetNode("/controls/flight", true);
    engine_node = pyGetNode("/controls/engine", true);
}


void control_init() {
    // initialize the autopilot class and build the structures from the
    // configuration file values

    bind_properties();

    remote_link_skip = remote_link_node.getDouble("autopilot_skip");
    logging_skip = logging_node.getDouble("autopilot_skip");

    // initialize the navigation module
    navigation.init("control.navigation");
    
    // initialize and build the autopilot controller from the property
    // tree config (/config/autopilot)
    ap.init();

    if ( display_on ) {
	printf("Autopilot initialized\n");
    }
}


// send a reset signal to all ap modules that support it.  This gives each
// component a chance to update it's state to reset for current conditions,
// eliminate transients, etc.
void control_reset() {
    events->log("controls", "global reset called");
    ap.reset();
}

static void copy_pilot_inputs() {
    // This function copies the pilot inputs to the flight/engine
    // outputs.  This creates a manual pass through mode.  Consider
    // that manaul pass-through is handled with less latency directly
    // on APM2/BFS/Aura3 hardware if available.
    
    float aileron = pilot_node.getDouble("aileron");
    flight_node.setDouble( "aileron", aileron );

    float elevator = pilot_node.getDouble("elevator");
    flight_node.setDouble( "elevator", elevator );

    float rudder = pilot_node.getDouble("rudder");
    flight_node.setDouble( "rudder", rudder );

    double flaps = pilot_node.getDouble("flaps");
    flight_node.setDouble("flaps", flaps );

    double gear = pilot_node.getDouble("gear");
    flight_node.setDouble("gear", gear );

    double throttle = pilot_node.getDouble("throttle");
    engine_node.setDouble("throttle", throttle );
}


void control_update(double dt)
{
    static int remote_link_count = 0;
    static int logging_count = 0;

    // call for a global fcs component reset when activating ap master
    // switch
    static bool last_master_switch = false;
    bool master_switch = ap_node.getBool("master_switch");
    if ( master_switch != last_master_switch ) {
	if ( ap_node.getBool("master_switch") ) {
            control_reset();    // transient mitigation
	}
	last_master_switch = ap_node.getBool("master_switch");
    }
    
    // update tecs (total energy) values and error metrics
    update_tecs();

    // navigation update (circle or route heading)
    navigation.update(dt);
        
    // update the autopilot stages (even in manual flight mode.)  This
    // keeps the differential value up to date, tracks manual inputs,
    // and keeps more continuity in the flight when the mode is
    // switched to autopilot.
    ap.update( dt );

    // copy pilot inputs to flight control outputs with not in
    // autopilot mode or in a pilot_pass_through mode
    bool pass_through = ap_node.getBool("pilot_pass_through");
    if ( !master_switch or pass_through ) {
        copy_pilot_inputs();
    }
    
    bool send_remote_link = false;
    if ( remote_link_count < 0 ) {
	send_remote_link = true;
	remote_link_count = remote_link_skip;
    }
	
    bool send_logging = false;
    if ( logging_count < 0 ) {
	send_logging = true;
	logging_count = logging_skip;
    }
	
    if ( send_remote_link || send_logging ) {
	uint8_t buf[256];
	int pkt_size = packer->pack_ap( 0, buf );
	
	if ( send_remote_link ) {
	    remote_link->send_message( buf, pkt_size );
	    // do the counter dance with the packer (packer will reset
	    // the count to zero at the appropriate time.)
	    int counter = comms_node.getLong("wp_counter");
	    counter++;
	    comms_node.setLong("wp_counter", counter);
	}

	if ( send_logging ) {
	    logging->log_message( buf, pkt_size );
	}
    }
    
    remote_link_count--;
    logging_count--;
}


void control_close() {
  // nothing to see here, move along ...
}
