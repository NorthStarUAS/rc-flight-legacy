//
// imu_mgr.hxx - front end IMU sensor management interface
//
// Written by Curtis Olson, curtolson <at> gmail <dot> com.  Spring 2009.
// This code is released into the public domain.
// 

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "include/ugear_config.h"

#include "comms/console_link.h"
#include "comms/logging.h"
#include "comms/packetizer.hxx"
#include "include/globaldefs.h"
#include "main/globals.hxx"
#include "props/props.hxx"
#include "util/myprof.h"

#include "sensors/imu_fgfs.hxx"
#ifdef ENABLE_MNAV_SENSOR
#  include "sensors/mnav.h"
#endif // ENABLE_MNAV_SENSOR
#include "sensors/ugfile.h"

#include "imu_mgr.hxx"


//
// Global variables
//

// comm property nodes
static SGPropertyNode *imu_console_skip = NULL;
static SGPropertyNode *imu_logging_skip = NULL;


void IMU_init() {
    // initialize comm nodes
    imu_console_skip = fgGetNode("/config/console/imu-skip", true);
    imu_logging_skip = fgGetNode("/config/logging/imu-skip", true);

    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/sensors/imu-group", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "imu" ) {
	    string source = section->getChild("source", 0, true)->getStringValue();
	    string basename = "/sensors/";
	    basename += section->getDisplayName();
	    printf("i = %d  name = %s source = %s %s\n",
		   i, name.c_str(), source.c_str(), basename.c_str());
	    if ( source == "null" ) {
		// do nothing
	    } else if ( source == "fgfs" ) {
		fgfs_imu_init( basename, section );
	    } else if ( source == "file" ) {
		ugfile_imu_init( basename, section );
#ifdef ENABLE_MNAV_SENSOR
	    } else if ( source == "mnav" ) {
		mnav_imu_init( basename, section );
#endif // ENABLE_MNAV_SENSOR
	    } else {
		printf("Unknown imu source = '%s' in config file\n",
		       source.c_str());
	    }
	}
    }
}


bool IMU_update() {
    imu_prof.start();

    bool fresh_data = false;

    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/sensors/imu-group", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "imu" ) {
	    string source = section->getChild("source", 0, true)->getStringValue();
	    // printf("i = %d  name = %s source = %s\n",
	    // 	   i, name.c_str(), source.c_str());
	    if ( source == "null" ) {
		// do nothing
	    } else if ( source == "fgfs" ) {
		fresh_data = fgfs_imu_update();
	    } else if ( source == "file" ) {
		ugfile_read();
		fresh_data = ugfile_get_imu();
#ifdef ENABLE_MNAV_SENSOR
	    } else if ( source == "mnav" ) {
		// read IMU until no data available.  This will flush any
		// potential backlog that could accumulate for any reason.
		mnav_start_nonblock_read();
		while ( mnav_read_nonblock() );
		fresh_data = mnav_get_imu();
#endif // ENABLE_MNAV_SENSOR
	    } else {
		printf("Unknown imu source = '%s' in config file\n",
		       source.c_str());
	    }
	}
    }

    imu_prof.stop();

    if ( fresh_data ) {
	if ( console_link_on || log_to_file ) {
	    uint8_t buf[256];
	    int size = packetizer->packetize_imu( buf );

	    if ( console_link_on ) {
		console_link_imu( buf, size, imu_console_skip->getDoubleValue() );
	    }

	    if ( log_to_file ) {
		log_imu( buf, size, imu_logging_skip->getDoubleValue() );
	    }
	}
    }

    return fresh_data;
}


void IMU_close() {
    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/sensors/imu-group", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "imu" ) {
	    string source = section->getChild("source", 0, true)->getStringValue();
	    printf("i = %d  name = %s source = %s\n",
		   i, name.c_str(), source.c_str());
	    if ( source == "null" ) {
		// do nothing
	    } else if ( source == "fgfs" ) {
		fgfs_imu_close();
	    } else if ( source == "file" ) {
		ugfile_close();
#ifdef ENABLE_MNAV_SENSOR
	    } else if ( source == "mnav" ) {
		// nop
#endif // ENABLE_MNAV_SENSOR
	    } else {
		printf("Unknown imu source = '%s' in config file\n",
		       source.c_str());
	    }
	}
    }
}
