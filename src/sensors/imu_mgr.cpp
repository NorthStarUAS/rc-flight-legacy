/**
 * \file: imu_mgr.cpp
 *
 * Front end management interface for reading IMU data.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@gmail.com
 *
 * $Id: imu_mgr.cpp,v 1.4 2009/08/25 15:04:02 curt Exp $
 */


#include <math.h>
#include <stdio.h>
#include <string.h>

#include "globaldefs.h"

#include "adns/mnav/ahrs.h"
#include "comms/console_link.h"
#include "comms/logging.h"
#include "props/props.hxx"
#include "util/myprof.h"

#include "mnav.h"
#include "ugfile.h"

#include "imu_mgr.h"

//
// Global variables
//


void IMU_init() {
    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/sensors", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "imu" ) {
	    string source = section->getChild("source")->getStringValue();
	    string basename = "/sensors/";
	    basename += section->getDisplayName();
	    printf("i = %d  name = %s source = %s %s\n",
		   i, name.c_str(), source.c_str(), basename.c_str());
	    if ( source == "file" ) {
		ugfile_imu_init( basename, section );
	    } else if ( source == "mnav" ) {
		mnav_imu_init( basename, section );
	    } else {
		printf("Unknown imu source = '%s' in config file\n",
		       source.c_str());
	    }
	}
    }
}


bool IMU_update() {
    struct imu imupacket;
    bool fresh_data = false;

    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/sensors", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "imu" ) {
	    string source = section->getChild("source")->getStringValue();
	    // printf("i = %d  name = %s source = %s\n",
	    // 	   i, name.c_str(), source.c_str());
	    if ( source == "file" ) {
		ugfile_read();
		fresh_data = ugfile_get_imu();
	    } else if ( source == "mnav" ) {
		mnav_prof.start();
		// read IMU until no data available.  This will flush any
		// potential backlog that could accumulate for any reason.
		mnav_start_nonblock_read();
		while ( mnav_read_nonblock() );
		mnav_prof.stop();
		fresh_data = mnav_get_imu();
	    } else {
		printf("Unknown imu source = '%s' in config file\n",
		       source.c_str());
	    }
	}
    }

    if ( fresh_data ) {
	if ( console_link_on ) {
	    console_link_imu( &imupacket );
	}

	if ( log_to_file ) {
	    log_imu( &imupacket );
	}
    }

    return fresh_data;
}


void IMU_close() {
    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/sensors", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "imu" ) {
	    string source = section->getChild("source")->getStringValue();
	    printf("i = %d  name = %s source = %s\n",
		   i, name.c_str(), source.c_str());
	    if ( source == "file" ) {
		ugfile_close();
	    } else if ( source == "mnav" ) {
		// nop
	    } else {
		printf("Unknown imu source = '%s' in config file\n",
		       source.c_str());
	    }
	}
    }
}
