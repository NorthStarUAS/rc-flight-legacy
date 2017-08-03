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
static pyPropertyNode ap_locks_node;
static pyPropertyNode targets_node;
static pyPropertyNode pointing_node;
static pyPropertyNode pointing_vec_node;
static pyPropertyNode orient_node;
static pyPropertyNode remote_link_node;
static pyPropertyNode logging_node;
static pyPropertyNode task_node;
static pyPropertyNode home_node;
static pyPropertyNode comms_node;

static int remote_link_skip = 0;
static int logging_skip = 0;

static void bind_properties() {
    ap_node = pyGetNode( "/autopilot", true );
    ap_locks_node = pyGetNode( "/autopilot/locks", true );
    targets_node = pyGetNode( "/autopilot/targets", true );
    pointing_node = pyGetNode( "/pointing", true );
    pointing_vec_node = pyGetNode( "/pointing/vector", true );
    orient_node = pyGetNode( "/orientation", true );
    remote_link_node = pyGetNode( "/config/remote_link", true );
    logging_node = pyGetNode( "/config/logging", true );
    task_node = pyGetNode( "/task", true );
    home_node = pyGetNode( "/task/home", true );
    comms_node = pyGetNode( "/comms/remote_link", true);
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
    printf("control reset\n");
    ap.reset();
}


void control_update(double dt)
{
    static int remote_link_count = 0;
    static int logging_count = 0;

    // manage control system component reset cases (make this
    // implicite per component when enable flag changes?)
    static bool last_master_switch = false;
    bool master_switch = ap_node.getBool("master_switch");
    if ( master_switch != last_master_switch ) {
	if ( ap_node.getBool("master_switch") ) {
            control_reset();    // transient mitigation
	}
	last_master_switch = ap_node.getBool("master_switch");
    }
    
    static string last_fcs_mode = "";
    string fcs_mode = ap_node.getString("mode");
    if ( master_switch ) {
	if ( last_fcs_mode != fcs_mode ) {
            control_reset();    // transient mitigation
	}
	last_fcs_mode = fcs_mode;
    }

    // update tecs (total energy) values and error metrics
    update_tecs();

    // navigation update (circle or route heading)
    navigation.update(dt);
        
    // update the autopilot stages (even in manual flight mode.)  This
    // keeps the differential metric up to date, tracks manual inputs,
    // and keeps more continuity in the flight when the mode is
    // switched to autopilot.
    ap.update( dt );
    
    // FIXME !!!
    // I want a departure route, an approach route, and mission route,
    // and circle hold point (all indicated on the ground station map.)
    // FIXME !!!

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
