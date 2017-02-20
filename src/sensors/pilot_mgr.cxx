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

#include <sstream>
#include <string>
#include <vector>
using std::ostringstream;
using std::string;
using std::vector;

#include "comms/logging.hxx"
#include "comms/remote_link.hxx"
#include "include/globaldefs.h"
#include "init/globals.hxx"
#include "util/myprof.hxx"

#include "APM2.hxx"
#include "Aura3.hxx"
#include "FGFS.hxx"
#include "Goldy2.hxx"
#include "pika.hxx"

#include "pilot_mgr.hxx"

//
// Global variables
//

// property nodes
static pyPropertyNode pilot_node;
static pyPropertyNode flight_node;
static pyPropertyNode engine_node;
static pyPropertyNode ap_node;
static vector<pyPropertyNode> sections;

static int remote_link_skip = 0;
static int logging_skip = 0;

void PilotInput_init() {
    pilot_node = pyGetNode("/sensors/pilot_input", true);
    flight_node = pyGetNode("/controls/flight", true);
    engine_node = pyGetNode("/controls/engine", true);
    ap_node = pyGetNode("/autopilot", true);
    
    pyPropertyNode remote_link_node = pyGetNode("/config/remote_link", true);
    pyPropertyNode logging_node = pyGetNode("/config/logging", true);
    remote_link_skip = remote_link_node.getDouble("pilot_skip");
    logging_skip = logging_node.getDouble("pilot_skip");

    // traverse configured modules
    pyPropertyNode group_node = pyGetNode("/config/sensors/pilot_inputs", true);
    vector<string> children = group_node.getChildren();
    printf("Found %d pilot sections\n", (int)children.size());
    for ( unsigned int i = 0; i < children.size(); i++ ) {
	pyPropertyNode section = group_node.getChild(children[i].c_str());
	sections.push_back(section);
	string source = section.getString("source");
	bool enabled = section.getBool("enable");
	if ( !enabled ) {
	    continue;
	}
	ostringstream output_path;
	output_path << "/sensors/pilot_input" << '[' << i << ']';
	printf("pilot: %d = %s\n", i, source.c_str());
	if ( source == "null" ) {
	    // do nothing
	} else if ( source == "APM2" ) {
	    APM2_pilot_init( output_path.str(), &section );
	} else if ( source == "Aura3" ) {
	    Aura3_pilot_init( output_path.str(), &section );
	} else if ( source == "fgfs" ) {
	    fgfs_pilot_init( output_path.str(), &section );
	} else if ( source == "Goldy2" ) {
	    goldy2_pilot_init( output_path.str(), &section );
	} else if ( source == "pika" ) {
	    pika_pilot_init( output_path.str(), &section );
	} else {
	    printf("Unknown pilot input source = '%s' in config file\n",
		   source.c_str());
	}
    }
}


bool PilotInput_update() {
    pilot_prof.start();

    bool fresh_data = false;

    static int remote_link_count = remote_link_random( remote_link_skip );
    static int logging_count = remote_link_random( logging_skip );

    // traverse configured modules
    for ( unsigned int i = 0; i < sections.size(); i++ ) {
	string source = sections[i].getString("source");
	bool enabled = sections[i].getBool("enable");
	if ( !enabled ) {
	    continue;
	}
	if ( source == "null" ) {
	    // do nothing
	} else if ( source == "APM2" ) {
	    fresh_data = APM2_pilot_update();
	} else if ( source == "Aura3" ) {
	    fresh_data = Aura3_pilot_update();
	} else if ( source == "fgfs" ) {
	    fresh_data = fgfs_pilot_update();
	} else if ( source == "Goldy2" ) {
	    fresh_data = goldy2_pilot_update();
	} else if ( source == "pika" ) {
	    fresh_data = pika_pilot_update();
	} else {
	    printf("Unknown pilot input source = '%s' in config file\n",
		   source.c_str());
	}
	if ( fresh_data ) {
	    bool send_remote_link = false;
	    if ( remote_link_on && remote_link_count < 0 ) {
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
		int size = packer->pack_pilot( i, buf );
		if ( send_remote_link ) {
		    remote_link_message( buf, size );
		}
		if ( send_logging ) {
		    logging->log_message( buf, size );
		}
	    }
	}
    }

    if ( fresh_data ) {
        // log receiver fail safe changes
        static bool last_fail_safe = false;
        if ( pilot_node.getBool("fail_safe") != last_fail_safe ) {
            char buf[128];
            snprintf( buf, 32, "Receiver fail safe = %d",
                      pilot_node.getBool("fail_safe") );
            events->log("Aura3", buf );
            last_fail_safe = pilot_node.getBool("fail_safe");
        }
        
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
	    flight_node.setDouble( "flaps", pilot_node.getDouble("flaps") );
	}

	if ( remote_link_on ) {
	    remote_link_count--;
	}
	
        logging_count--;
    }

    pilot_prof.stop();

    return fresh_data;
}


void PilotInput_close() {
    // traverse configured modules
    for ( unsigned int i = 0; i < sections.size(); i++ ) {
	string source = sections[i].getString("source");
	bool enabled = sections[i].getBool("enable");
	if ( !enabled ) {
	    continue;
	}
	if ( source == "null" ) {
	    // do nothing
	} else if ( source == "APM2" ) {
	    APM2_pilot_close();
	} else if ( source == "Aura3" ) {
	    Aura3_pilot_close();
	} else if ( source == "fgfs" ) {
	    fgfs_pilot_close();
	} else if ( source == "Goldy2" ) {
	    goldy2_pilot_close();
	} else if ( source == "pika" ) {
	    pika_pilot_close();
	} else {
	    printf("Unknown pilot input source = '%s' in config file\n",
		   source.c_str());
	}
    }
}
