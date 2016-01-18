/**
 * \file: pilot_mgr.cxx
 *
 * Front end management interface for reading pilot input.
 *
 * Copyright (C) 2010 - Curtis L. Olson curtolson@flightgear.org
 *
 */


#include "python/pyprops.hxx"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "comms/logging.hxx"
#include "comms/remote_link.hxx"
#include "include/globaldefs.h"
#include "init/globals.hxx"
#include "util/myprof.h"

#include "APM2.hxx"
#include "Goldy2.hxx"
#include "pilot_fgfs.hxx"

#include "pilot_mgr.hxx"

//
// Global variables
//

// property nodes
static pyPropertyNode pilot_node;
static pyPropertyNode flight_node;
static pyPropertyNode engine_node;
static pyPropertyNode remote_link_node;
static pyPropertyNode logging_node;
static pyPropertyNode ap_node;


void PilotInput_init() {
    // traverse configured modules
    pyPropertyNode group_node = pyGetNode("/config/sensors/pilot_inputs", true);
    vector<string> children = group_node.getChildren();
    printf("Found %d pilot sections\n", children.size());
    for ( unsigned int i = 0; i < children.size(); i++ ) {
	pyPropertyNode section = group_node.getChild(children[i].c_str());
	string source = section.getString("source");
	bool enabled = section.getBool("enable");
	if ( !enabled ) {
	    continue;
	}
	pyPropertyNode parent = pyGetNode("/sensors/", true);
	pyPropertyNode base = parent.getChild("pilot_input", i, true);
	printf("pilot: %d = %s\n", i, source.c_str());
	if ( source == "null" ) {
	    // do nothing
	} else if ( source == "APM2" ) {
	    APM2_pilot_init( &base );
	} else if ( source == "fgfs" ) {
	    fgfs_pilot_init( &base, &section );
	} else if ( source == "Goldy2" ) {
	    goldy2_pilot_init( &base, &section );
	} else {
	    printf("Unknown pilot input source = '%s' in config file\n",
		   source.c_str());
	}
    }
}


bool PilotInput_update() {
    pilot_prof.start();

    bool fresh_data = false;

    // traverse configured modules
    pyPropertyNode group_node = pyGetNode("/config/sensors/pilot_inputs", true);
    vector<string> children = group_node.getChildren();
    for ( unsigned int i = 0; i < children.size(); i++ ) {
	pyPropertyNode section = group_node.getChild(children[i].c_str());
	string source = section.getString("source");
	bool enabled = section.getBool("enable");
	if ( !enabled ) {
	    continue;
	}
	if ( source == "null" ) {
	    // do nothing
	} else if ( source == "APM2" ) {
	    fresh_data = APM2_pilot_update();
	} else if ( source == "fgfs" ) {
	    fresh_data = fgfs_pilot_update();
	} else if ( source == "Goldy2" ) {
	    fresh_data = goldy2_pilot_update();
	} else {
	    printf("Unknown pilot input source = '%s' in config file\n",
		   source.c_str());
	}
    }

    if ( fresh_data ) {

	// Hello this is a bit of a hack to hard code the master
	// autopilot on/off switch here.  In the future the master
	// autopilot on/off switch may come from other sources. (?)
	ap_node.setBool( "master_switch", !pilot_node.getBool("manual") );
	// if ( display_on ) {
	//    printf("autopilot = %d\n", ap_master_switch_node->getBool());
	// }

	// Only in manual mode, do copy the pilot inputs to the main
	// AP outputs.  This puts the pilot inputs in a standard place
	// and allows the AP to seed it's components with trimmed
	// values and improve continuity when switching from manual to
	// AP mode.
	if ( ! ap_node.getBool("master_switch") ) {
	    flight_node.setDouble( "aileron", pilot_node.getDouble("aileron") );
	    flight_node.setDouble( "elevator", pilot_node.getDouble("elevator") );
	    engine_node.setDouble( "throttle", pilot_node.getDouble("throttle") );
	    flight_node.setDouble( "rudder", pilot_node.getDouble("rudder") );
	}

	if ( remote_link_on || log_to_file ) {
	    uint8_t buf[256];
	    int size = packetizer->packetize_pilot( buf );

	    if ( remote_link_on ) {
		// printf("sending filter packet\n");
		remote_link_pilot( buf, size,
				    remote_link_node.getLong("pilot_skip") );
	    }

	    if ( log_to_file ) {
		log_pilot( buf, size, logging_node.getLong("pilot_skip") );
	    }
	}
    }

    pilot_prof.stop();

    return fresh_data;
}


void PilotInput_close() {
    // traverse configured modules
    pyPropertyNode group_node = pyGetNode("/config/sensors/pilot_inputs", true);
    vector<string> children = group_node.getChildren();
    for ( unsigned int i = 0; i < children.size(); i++ ) {
	pyPropertyNode section = group_node.getChild(children[i].c_str());
	string source = section.getString("source");
	bool enabled = section.getBool("enable");
	if ( !enabled ) {
	    continue;
	}
	if ( source == "null" ) {
	    // do nothing
	} else if ( source == "APM2" ) {
	    APM2_pilot_close();
	} else if ( source == "fgfs" ) {
	    fgfs_pilot_close();
	} else if ( source == "Goldy2" ) {
	    goldy2_pilot_close();
	} else {
	    printf("Unknown pilot input source = '%s' in config file\n",
		   source.c_str());
	}
    }
}
