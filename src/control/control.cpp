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

#include <pyprops.h>

#include <stdio.h>
#include <sys/time.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

#include <sstream>
#include <string>
using std::ostringstream;
using std::string;

#include "comms/aura_messages.h"
#include "comms/display.h"
#include "comms/logging.h"
#include "comms/remote_link.h"
#include "include/globaldefs.h"
#include "init/globals.h"
#include <pymodule.h>

#include "include/util.h"
#include "ap.h"
#include "tecs.h"

#include "control.h"


// global variables
static pyModuleBase navigation;
static AuraAutopilot ap;


// property nodess
static pyPropertyNode status_node;
static pyPropertyNode ap_node;
static pyPropertyNode targets_node;
static pyPropertyNode tecs_node;
static pyPropertyNode task_node;
static pyPropertyNode logging_node;
static pyPropertyNode remote_link_node;
static pyPropertyNode pilot_node;
static pyPropertyNode flight_node;
static pyPropertyNode engine_node;
static pyPropertyNode route_node;
static pyPropertyNode active_node;
static pyPropertyNode home_node;
static pyPropertyNode circle_node;
static pyPropertyNode pos_node;

static int remote_link_skip = 0;
static int logging_skip = 0;

static void bind_properties() {
    status_node = pyGetNode( "/status", true );
    ap_node = pyGetNode( "/autopilot", true );
    targets_node = pyGetNode( "/autopilot/targets", true );
    tecs_node = pyGetNode( "/autopilot/tecs", true );
    task_node = pyGetNode( "/task", true );
    remote_link_node = pyGetNode( "/comms/remote_link", true );
    logging_node = pyGetNode( "/config/logging", true );
    pilot_node = pyGetNode("/sensors/pilot_input", true);
    flight_node = pyGetNode("/controls/flight", true);
    engine_node = pyGetNode("/controls/engine", true);
    route_node = pyGetNode("/task/route", true);
    active_node = pyGetNode("/task/route/active", true);
    home_node = pyGetNode("/task/home", true);
    circle_node = pyGetNode("/task/circle", true);
    pos_node = pyGetNode("/position", true);
}


void control_init() {
    // initialize the autopilot class and build the structures from the
    // configuration file values

    bind_properties();

    pyPropertyNode remote_link_config = pyGetNode( "/config/remote_link", true );
    remote_link_skip = remote_link_config.getDouble("autopilot_skip");
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
        message::ap_status_v7_t ap;
        ap.index = 0;     // always 0 for now
        ap.timestamp_sec = status_node.getDouble("frame_time");
        // status flags (up to 8 could be supported)
        ap.flags = 0;
        if ( ap_node.getBool("master_switch") ) {
            ap.flags |= (1 << 0);
        }
        if ( ap_node.getBool("pilot_pass_through") ) {
            ap.flags |= (1 << 1);
        }
        ap.groundtrack_deg = targets_node.getDouble("groundtrack_deg");
        ap.roll_deg = targets_node.getDouble("roll_deg");
        float target_agl_ft = targets_node.getDouble("altitude_agl_ft");
        float ground_m = pos_node.getDouble("altitude_ground_m");
        // if ( pressure based ) {
        //   error_m = pos_pressure_node.getDouble("pressure_error_m")
        //   target_msl_ft = (ground_m + error_m) * m2ft + target_agl_ft
        // } else { ...
        ap.altitude_msl_ft = ground_m * SG_METER_TO_FEET + target_agl_ft;
        ap.altitude_ground_m = ground_m;
        ap.pitch_deg = targets_node.getDouble("pitch_deg");
        ap.airspeed_kt = targets_node.getDouble("airspeed_kt");
        ap.flight_timer = task_node.getDouble("flight_timer");
        ap.target_waypoint_idx = route_node.getLong("target_waypoint_idx");

        double wp_lon = 0.0;
        double wp_lat = 0.0;
        int wp_index = 0;
        uint16_t task_attr = 0;
        int counter = remote_link_node.getLong("wp_counter");
        ap.route_size = active_node.getLong("route_size");
        if ( ap.route_size > 0 and counter < ap.route_size ) {
            wp_index = counter;
            ostringstream wp_path;
            wp_path << "wpt[" << wp_index << "]";
            pyPropertyNode wp_node = active_node.getChild(wp_path.str().c_str());
            if ( !wp_node.isNull() ) {
                wp_lon = wp_node.getDouble("longitude_deg");
                wp_lat = wp_node.getDouble("latitude_deg");
            }
        } else if ( counter == ap.route_size ) {
            wp_lon = circle_node.getDouble("longitude_deg");
            wp_lat = circle_node.getDouble("latitude_deg");
            wp_index = 65534;
            task_attr = int(round(circle_node.getDouble("radius_m") * 10));
            if ( task_attr > 32767 ) { task_attr = 32767; }
        } else if ( counter == ap.route_size + 1 ) {
            wp_lon = home_node.getDouble("longitude_deg");
            wp_lat = home_node.getDouble("latitude_deg");
            wp_index = 65535;
        }
        ap.wp_longitude_deg = wp_lon;
        ap.wp_latitude_deg = wp_lat;
        ap.wp_index = wp_index;
        
        ap.task_id = 0;         // code for unknown or not set
        if ( task_node.getString("current_task_id") == "circle" ) {
            ap.task_id = 1;
        } else if ( task_node.getString("current_task_id") == "parametric" ) {
            // draw like it's a circle
            ap.task_id = 1;
        } else if ( task_node.getString("current_task_id") == "route" ) {
            ap.task_id = 2;
        } else if ( task_node.getString("current_task_id") == "land" ) {
            ap.task_id = 3;
        }
        ap.task_attribute = task_attr;
        ap.sequence_num = remote_link_node.getLong("sequence_num");
        ap.pack();
	if ( send_remote_link ) {
	    remote_link->send_message( ap.id, ap.payload, ap.len );
            
	    // do the counter increment here (only when sending a
	    // telemetry packet)
	    counter++;
            if ( counter >= ap.route_size + 2 ) {
                counter = 0;
            }
            remote_link_node.setLong("wp_counter", counter);
	}

	if ( send_logging ) {
	    logging->log_message( ap.id, ap.payload, ap.len );
	}
    }
    
    remote_link_count--;
    logging_count--;
}


void control_close() {
  // nothing to see here, move along ...
}
