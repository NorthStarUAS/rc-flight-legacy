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
#include "comms/packetizer.hxx"
#include "comms/remote_link.hxx"
#include "include/globaldefs.h"
#include "init/globals.hxx"

#include "include/util.h"
#include "xmlauto.hxx"

#include "control.hxx"


//
// global variables
//

// the "FlightGear" autopilot
static FGXMLAutopilot ap;


// property nodes
static pyPropertyNode ap_node;
static pyPropertyNode ap_locks_node;
static pyPropertyNode ap_settings_node;
static pyPropertyNode fcs_node;
static pyPropertyNode pointing_node;
static pyPropertyNode pointing_vec_node;
static pyPropertyNode orient_node;
static pyPropertyNode remote_link_node;
static pyPropertyNode logging_node;
static pyPropertyNode task_node;
static pyPropertyNode home_node;


static void bind_properties() {
    ap_node = pyGetNode( "/autopilot", true );
    ap_locks_node = pyGetNode( "/autopilot/locks", true );
    ap_settings_node = pyGetNode( "/autopilot/settings", true );
    fcs_node = pyGetNode( "/config/autopilot", true );
    pointing_node = pyGetNode( "/pointing", true );
    pointing_vec_node = pyGetNode( "/pointing/vector", true );
    orient_node = pyGetNode( "/orientation", true );
    remote_link_node = pyGetNode( "/config/remote_link", true );
    logging_node = pyGetNode( "/config/logging", true );
    task_node = pyGetNode( "/task", true );
    home_node = pyGetNode( "/task/home", true );    
}


void control_init() {
    // initialize the autopilot class and build the structures from the
    // configuration file values

    bind_properties();

    circle_mgr->init();
    route_mgr->init();
    
    // initialize and build the autopilot controller from the property
    // tree config (/config/autopilot)
    ap.init();

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
    // FIXME: there's probably a better place than this, but we need
    // to update the pattern routes every frame (even if the route
    // task is not active) and so the code to do this is going here
    // for now.
    route_mgr->reposition_if_necessary();

    if ( task_node.getString("current_task_id") == "circle" ) {
	circle_mgr->update();
    } else if ( task_node.getString("current_task_id") == "route" ) {
	route_mgr->update();
    }
    
    // log auto/manual mode changes
    static bool last_ap_mode = false;
    if ( ap_node.getBool("master_switch") != last_ap_mode ) {
	string ap_master_str;
	if ( ap_node.getBool("master_switch") ) {
	    ap_master_str = "autopilot";
	} else {
	    ap_master_str = "manual flight";
	}
	string message = "master switch = " + ap_master_str;
	events->log( "control", message.c_str() );
	last_ap_mode = ap_node.getBool("master_switch");
    }
    
    static string last_fcs_mode = "";
    string fcs_mode = fcs_node.getString("mode");
    if ( ap_node.getBool("master_switch") ) {
	if ( last_fcs_mode != fcs_mode ) {
	    string message = "mode change = " + fcs_mode;
	    events->log( "control", message.c_str() );

	    // turn on pointing (universally for now)
	    ap_locks_node.setString( "pointing", "on" );
	    pointing_node.setString( "lookat_mode", "ned-vector" );
	    pointing_vec_node.setDouble( "north", 0.0 );
	    pointing_vec_node.setDouble( "east", 0.0 );
	    pointing_vec_node.setDouble( "down", 1.0 );

	    if ( fcs_mode == "inactive" ) {
		// unset all locks for "inactive"
		ap_locks_node.setString( "roll", "" );
		ap_locks_node.setString( "yaw", "" );
		ap_locks_node.setString( "altitude", "" );
		ap_locks_node.setString( "speed", "" );
		ap_locks_node.setString( "pitch", "" );
	    } else if ( fcs_mode == "basic" ) {
		// set lock modes for "basic" inner loops only
		ap_locks_node.setString( "roll", "aileron" );
		ap_locks_node.setString( "yaw", "autocoord" );
		ap_locks_node.setString( "altitude", "" );
		ap_locks_node.setString( "speed", "" );
		ap_locks_node.setString( "pitch", "elevator" );
	    } else if ( fcs_mode == "roll" ) {
		// set lock modes for roll only
		ap_locks_node.setString( "roll", "aileron" );
		ap_locks_node.setString( "yaw", "" );
		ap_locks_node.setString( "altitude", "" );
		ap_locks_node.setString( "speed", "" );
		ap_locks_node.setString( "pitch", "" );
	    } else if ( fcs_mode == "roll+pitch" ) {
		// set lock modes for roll and pitch
		ap_locks_node.setString( "roll", "aileron" );
		ap_locks_node.setString( "yaw", "" );
		ap_locks_node.setString( "altitude", "" );
		ap_locks_node.setString( "speed", "" );
		ap_locks_node.setString( "pitch", "elevator" );
	    } else if ( fcs_mode == "basic+alt+speed" ) {
		// set lock modes for "basic" + alt hold
		ap_locks_node.setString( "roll", "aileron" );
		ap_locks_node.setString( "yaw", "autocoord" );
		ap_locks_node.setString( "altitude", "throttle" );
		ap_locks_node.setString( "speed", "pitch" );
		ap_locks_node.setString( "pitch", "elevator" );
	    } else if ( fcs_mode == "cas" ) {
		// set lock modes for "cas"
		ap_locks_node.setString( "roll", "aileron" );
		ap_locks_node.setString( "yaw", "" );
		ap_locks_node.setString( "altitude", "" );
		ap_locks_node.setString( "speed", "" );
		ap_locks_node.setString( "pitch", "elevator" );
		ap_locks_node.setString( "pointing", "on" );

		float target_roll_deg = orient_node.getDouble("roll_deg");
		if ( target_roll_deg > 45.0 ) { target_roll_deg = 45.0; }
		if ( target_roll_deg < -45.0 ) { target_roll_deg = -45.0; }
		ap_settings_node.setDouble( "target_roll_deg", target_roll_deg );

		float target_pitch_base_deg = orient_node.getDouble("pitch_deg");
		if ( target_pitch_base_deg > 15.0 ) {
		    target_pitch_base_deg = 15.0;
		}
		if ( target_pitch_base_deg < -15.0 ) {
		    target_pitch_base_deg = -15.0;
		}
		ap_settings_node.setDouble( "target_pitch_base_deg", target_pitch_base_deg );
	    }
	}
	last_fcs_mode = fcs_mode;
    } else {
	if ( fcs_mode != "" ) {
	    // autopilot is just de-activated, clear lock modes
	    ap_locks_node.setString( "roll", "" );
	    ap_locks_node.setString( "yaw", "" );
	    ap_locks_node.setString( "altitude", "" );
	    ap_locks_node.setString( "speed", "" );
	    ap_locks_node.setString( "pitch", "" );
	    ap_locks_node.setString( "pointing", "" );
	}
	last_fcs_mode = "";
    }

    // update the autopilot stages (even in manual flight mode.)  This
    // keeps the differential metric up to date, tracks manual inputs,
    // and keeps more continuity in the flight when the mode is
    // switched to autopilot.
    ap.update( dt );

    // FIXME !!!
    // I want a departure route, an approach route, and mission route,
    // and circle hold point (all indicated on the ground station map.)
    // FIXME !!!
    
    if ( remote_link_on || log_to_file ) {
	// send one waypoint per message, then home location (with
	// index = 65535)

	static int wp_index = 0;
	int index = 0;
	SGWayPoint wp;
	int route_size = 0;

	string task_name = task_node.getString("current_task_id");
	if ( task_name == "route" ) {
	    if ( route_mgr != NULL ) {
		route_size = route_mgr->size();
		if ( route_size > 0 && wp_index < route_size ) {
		    wp = route_mgr->get_waypoint( wp_index );
		    index = wp_index;
		}
	    }
	} else if ( task_name == "circle-coord" ) {
	    if ( circle_mgr != NULL ) {
		wp = circle_mgr->get_center();
		route_size = 1;
		index = wp_index;
	    }
	}

	// special case send home as a route waypoint with id = 65535
	if ( wp_index == route_size ) {
	    wp = SGWayPoint( home_node.getDouble("longitude_deg"),
			     home_node.getDouble("latitude_deg"),
			     home_node.getDouble("altitude_ft") );
	    index = 65535;
	}

	uint8_t buf[256];
	int pkt_size = packetizer->packetize_ap( buf, route_size, &wp, index );
	
	if ( remote_link_on ) {
	    bool result = remote_link_ap( buf, pkt_size,
					  remote_link_node.getLong("autopilot_skip") );
	    if ( result ) {
		wp_index++;
		if ( wp_index > route_size ) {
		    wp_index = 0;
		}
	    }
	}

	if ( log_to_file ) {
	    log_ap( buf, pkt_size, logging_node.getLong("autopilot_skip") );
	}
    }
}


void control_close() {
  // nothing to see here, move along ...
}
