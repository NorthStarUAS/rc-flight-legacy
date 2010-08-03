/**
 * \file: pilot_mgr.cpp
 *
 * Front end management interface for reading pilot input.
 *
 * Copyright (C) 2010 - Curtis L. Olson curtolson@gmail.com
 *
 * $Id: airdata_mgr.cpp,v 1.2 2009/08/25 15:04:01 curt Exp $
 */


#include <math.h>
#include <stdio.h>
#include <string.h>

#include "include/ugear_config.h"

#include "comms/console_link.h"
#include "comms/logging.h"
#include "include/globaldefs.h"
#include "main/globals.hxx"
#include "props/props.hxx"
#include "util/myprof.h"

#include "actuators/ardusensor.hxx"
#include "sensors/pilot_fgfs.hxx"

#include "pilot_mgr.h"

//
// Global variables
//

// pilot input property nodes
static SGPropertyNode *pilot_timestamp_node = NULL;
static SGPropertyNode *pilot_aileron_node = NULL;
static SGPropertyNode *pilot_elevator_node = NULL;
static SGPropertyNode *pilot_throttle_node = NULL;
static SGPropertyNode *pilot_rudder_node = NULL;
static SGPropertyNode *pilot_manual_node = NULL;
static SGPropertyNode *pilot_channel6_node = NULL;
static SGPropertyNode *pilot_channel7_node = NULL;
static SGPropertyNode *pilot_channel8_node = NULL;
static SGPropertyNode *pilot_status_node = NULL;

// comm property nodes
static SGPropertyNode *pilot_console_skip = NULL;
static SGPropertyNode *pilot_logging_skip = NULL;

// master autopilot switch
static SGPropertyNode *ap_master_switch_node = NULL;


void PilotInput_init() {
    // pilot input property nodes
    pilot_timestamp_node = fgGetNode("/controls/pilot/time-stamp", true);
    pilot_aileron_node = fgGetNode("/controls/pilot/aileron", 0, true);
    pilot_elevator_node = fgGetNode("/controls/pilot/elevator", 1, true);
    pilot_throttle_node = fgGetNode("/controls/pilot/throttle", 2, true);
    pilot_rudder_node = fgGetNode("/controls/pilot/rudder", 3, true);
    pilot_manual_node = fgGetNode("/controls/pilot/manual", 4, true);
    pilot_channel6_node = fgGetNode("/controls/pilot/channel", 5, true);
    pilot_channel7_node = fgGetNode("/controls/pilot/channel", 6, true);
    pilot_channel8_node = fgGetNode("/controls/pilot/channel", 7, true);
    pilot_status_node = fgGetNode("/controls/pilot/status", true);

    // initialize comm nodes
    pilot_console_skip = fgGetNode("/config/console/pilot-skip", true);
    pilot_logging_skip = fgGetNode("/config/logging/pilot-skip", true);

    // master autopilot switch
    ap_master_switch_node = fgGetNode("/autopilot/master-switch", true);

    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/sensors/pilot-inputs", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "pilot-control" ) {
	    string source = section->getChild("source", 0, true)->getStringValue();
	    bool enabled = section->getChild("enable", 0, true)->getBoolValue();
	    if ( !enabled ) {
		continue;
	    }
	    string basename = "/sensors/";
	    basename += section->getDisplayName();
	    printf("i = %d  name = %s source = %s %s\n",
		   i, name.c_str(), source.c_str(), basename.c_str());
	    if ( source == "null" ) {
		// do nothing
	    } else if ( source == "ardusensor" ) {
		ardusensor_pilot_init( basename );
	    } else if ( source == "fgfs" ) {
		fgfs_pilot_init( basename, section );
	    } else {
		printf("Unknown pilot input source = '%s' in config file\n",
		       source.c_str());
	    }
	}
    }
}


bool PilotInput_update() {
    pilot_prof.start();

    bool fresh_data = false;

    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/sensors/pilot-inputs", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "pilot-input" ) {
	    string source = section->getChild("source", 0, true)->getStringValue();
	    bool enabled = section->getChild("enable", 0, true)->getBoolValue();
	    if ( !enabled ) {
		continue;
	    }
	    string basename = "/sensors/";
	    basename += section->getDisplayName();
	    // printf("i = %d  name = %s source = %s %s\n",
	    //	   i, name.c_str(), source.c_str(), basename.c_str());
	    if ( source == "null" ) {
		// do nothing
	    } else if ( source == "ardusensor" ) {
		fresh_data = ardusensor_pilot_update();
	    } else if ( source == "fgfs" ) {
		fresh_data = fgfs_pilot_update();
	    } else {
		printf("Unknown air data source = '%s' in config file\n",
		       source.c_str());
	    }
	}
    }

    if ( fresh_data ) {

	// Hello this is a bit of a hack to hard code the master
	// autopilot on/off switch here.  In the future, actuators and
	// pilot inputs will need to be generalized into their own
	// separate modules and the master autopilot on/off switch may
	// come from other sources.
	ap_master_switch_node
	    ->setBoolValue( !pilot_manual_node->getBoolValue() );

	if ( console_link_on || log_to_file ) {
	    uint8_t buf[256];
	    int size = packetizer->packetize_pilot( buf );

	    if ( console_link_on ) {
		// printf("sending filter packet\n");
		console_link_pilot( buf, size,
				    pilot_console_skip->getIntValue() );
	    }

	    if ( log_to_file ) {
		log_pilot( buf, size, pilot_logging_skip->getIntValue() );
	    }
	}
    }

    pilot_prof.stop();

    return fresh_data;
}


void PilotInput_close() {
    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/sensors/pilot-inputs", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "pilot-input" ) {
	    string source = section->getChild("source", 0, true)->getStringValue();
	    string basename = "/sensors/";
	    basename += section->getDisplayName();
	    // printf("i = %d  name = %s source = %s %s\n",
	    //	   i, name.c_str(), source.c_str(), basename.c_str());
	    if ( source == "null" ) {
		// do nothing
	    } else if ( source == "fgfs" ) {
		fgfs_pilot_close();
	    } else if ( source == "ardusensor" ) {
		// nop
	    } else {
		printf("Unknown air data source = '%s' in config file\n",
		       source.c_str());
	    }
	}
    }
}
