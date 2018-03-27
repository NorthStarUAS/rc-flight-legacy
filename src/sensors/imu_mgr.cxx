//
// imu_mgr.hxx - front end IMU sensor management interface
//
// Written by Curtis Olson, curtolson <at> gmail <dot> com.  Spring 2009.
// This code is released into the public domain.
// 

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
#include "util/timing.h"

#include "sensors/APM2.hxx"
#include "sensors/Aura3/Aura3.hxx"
#include "sensors/FGFS.hxx"
#include "sensors/Goldy2.hxx"
#include "sensors/imu_vn100_spi.hxx"
#include "sensors/imu_vn100_uart.hxx"
#include "sensors/ugfile.hxx"

#include "imu_mgr.hxx"


//
// Global variables
//

static double imu_last_time = -31557600.0; // default to t minus one year old

static pyPropertyNode imu_node;
static vector<pyPropertyNode> sections;

static int remote_link_skip = 0;
static int logging_skip = 0;

static myprofile debug2a1;
static myprofile debug2a2;
	

void IMU_init() {
    debug2a1.set_name("debug2a1 IMU read");
    debug2a2.set_name("debug2a2 IMU console link");

    imu_node = pyGetNode("/sensors/imu", true);

    pyPropertyNode remote_link_node = pyGetNode("/config/remote_link", true);
    pyPropertyNode logging_node = pyGetNode("/config/logging", true);
    remote_link_skip = remote_link_node.getDouble("imu_skip");
    logging_skip = logging_node.getDouble("imu_skip");

    // traverse configured modules
    pyPropertyNode group_node = pyGetNode("/config/sensors/imu_group", true);
    vector<string>children = group_node.getChildren();
    printf("Found %d imu sections\n", (int)children.size());
    for ( unsigned int i = 0; i < children.size(); i++ ) {
	pyPropertyNode section = group_node.getChild(children[i].c_str());
	sections.push_back(section);
	string source = section.getString("source");
	bool enabled = section.getBool("enable");
	if ( !enabled ) {
	    continue;
	}
	ostringstream output_path;
	output_path << "/sensors/imu" << '[' << i << ']';
	printf("imu: %d = %s\n", i, source.c_str());
	if ( source == "null" ) {
	    // do nothing
	} else if ( source == "APM2" ) {
	    APM2_imu_init( output_path.str(), &section );
	} else if ( source == "Aura3" ) {
	    Aura3_imu_init( output_path.str(), &section );
	} else if ( source == "fgfs" ) {
	    fgfs_imu_init( output_path.str(), &section );
	} else if ( source == "file" ) {
	    ugfile_imu_init( output_path.str(), &section );
	} else if ( source == "Goldy2" ) {
	    goldy2_imu_init( output_path.str(), &section );
	} else if ( source == "vn100" ) {
	    imu_vn100_uart_init( output_path.str(), &section );
	} else if ( source == "vn100-spi" ) {
	    imu_vn100_spi_init( output_path.str(), &section );
	} else {
	    printf("Unknown imu source = '%s' in config file\n",
		   source.c_str());
	}
    }
}


bool IMU_update() {
    debug2a1.start();

    imu_prof.start();

    bool fresh_data = false;

    static int remote_link_count = 0;
    static int logging_count = 0;

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
	    fresh_data = APM2_imu_update();
	} else if ( source == "Aura3" ) {
	    fresh_data = Aura3_imu_update();
	} else if ( source == "fgfs" ) {
	    fresh_data = fgfs_imu_update();
	} else if ( source == "file" ) {
	    ugfile_read();
	    fresh_data = ugfile_get_imu();
	} else if ( source == "Goldy2" ) {
	    fresh_data = goldy2_imu_update();
	} else if ( source == "vn100" ) {
	    fresh_data = imu_vn100_uart_get();
	} else if ( source == "vn100-spi" ) {
	    fresh_data = imu_vn100_spi_get();
	} else {
	    printf("Unknown imu source = '%s' in config file\n",
		   source.c_str());
	}
	if ( fresh_data ) {
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
		int size = packer->pack_imu( i, buf );
		if ( send_remote_link ) {
		    remote_link->send_message( buf, size );
		}
		if ( send_logging ) {
		    logging->log_message( buf, size );
		}
	    }
	}
    }

    imu_prof.stop();
    debug2a1.stop();

    debug2a2.start();

    if ( fresh_data ) {
	// for computing imu data age
	imu_last_time = imu_node.getDouble("timestamp");

        remote_link_count--;
        logging_count--;
    }
    
    debug2a2.stop();

    return fresh_data;
}


void IMU_close() {
    // traverse configured modules
    for ( unsigned int i = 0; i < sections.size(); i++ ) {
	string source = sections[i].getString("source");
	bool enabled = sections[i].getBool("enable");
	//printf("i = %d  name = %s source = %s\n",
	//       i, name.c_str(), source.c_str());
	if ( !enabled ) {
	    continue;
	}
	if ( source == "null" ) {
	    // do nothing
	} else if ( source == "APM2" ) {
	    APM2_imu_close();
	} else if ( source == "Aura3" ) {
	    Aura3_imu_close();
	} else if ( source == "fgfs" ) {
	    fgfs_imu_close();
	} else if ( source == "file" ) {
	    ugfile_close();
	} else if ( source == "Goldy2" ) {
	    goldy2_imu_close();
	} else if ( source == "vn100" ) {
	    imu_vn100_uart_close();
	} else if ( source == "vn100-spi" ) {
	    imu_vn100_spi_close();
	} else {
	    printf("Unknown imu source = '%s' in config file\n",
		   source.c_str());
	}
    }
}


double IMU_age() {
    return get_Time() - imu_last_time;
}
