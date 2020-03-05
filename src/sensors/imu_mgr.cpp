//
// imu_mgr.cpp - front end IMU sensor management interface
//
// Written by Curtis Olson, curtolson <at> gmail <dot> com.  Spring 2009.
// This code is released into the public domain.
// 

#include <pyprops.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <sstream>
#include <string>
#include <vector>
using std::ostringstream;
using std::string;
using std::vector;

#include "comms/aura_messages.h"
#include "comms/logging.h"
#include "comms/remote_link.h"
#include "include/globaldefs.h"
#include "init/globals.h"
#include "util/myprof.h"
#include "util/timing.h"

#include "sensors/APM2.h"
#include "sensors/Aura3/Aura3.h"
#include "sensors/FGFS.h"
#include "sensors/imu_vn100_spi.h"
#include "sensors/imu_vn100_uart.h"
#include "sensors/ugfile.h"

#include "imu_mgr.h"


//
// Global variables
//

static double imu_last_time = -31557600.0; // default to t minus one year old

static pyPropertyNode imu_node;
static vector<pyPropertyNode> sections;
static vector<pyPropertyNode> outputs;

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
        pyPropertyNode output_node = pyGetNode(output_path.str(), true);
        outputs.push_back(output_node);
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
                // generate the message
                message::imu_v4_t imu;
                imu.index = i;
                imu.timestamp_sec = outputs[i].getDouble("timestamp");
                imu.p_rad_sec = outputs[i].getDouble("p_rad_sec");
                imu.q_rad_sec = outputs[i].getDouble("q_rad_sec");
                imu.r_rad_sec = outputs[i].getDouble("r_rad_sec");
                imu.ax_mps_sec = outputs[i].getDouble("ax_mps_sec");
                imu.ay_mps_sec = outputs[i].getDouble("ay_mps_sec");
                imu.az_mps_sec = outputs[i].getDouble("az_mps_sec");
                imu.hx = outputs[i].getDouble("hx");
                imu.hy = outputs[i].getDouble("hy");
                imu.hz = outputs[i].getDouble("hz");
                imu.temp_C = outputs[i].getDouble("temp_C");
                imu.status = 0;
                imu.pack();
		if ( send_remote_link ) {
		    remote_link->send_message( imu.id, imu.payload, imu.len );
		}
		if ( send_logging ) {
		    logging->log_message( imu.id, imu.payload, imu.len );
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
