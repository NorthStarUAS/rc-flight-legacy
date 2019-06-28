/**
 * \file: filter_mgr.cpp
 *
 * Front end management interface for executing the available filter codes.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson <at> gmail <dot> com
 *
 * $Id: adns_mgr.cpp,v 1.6 2009/05/15 17:04:56 curt Exp $
 */


#include <pyprops.hxx>

#include <stdio.h>
#include <string>
#include <sstream>
using std::string;
using std::ostringstream;

#include "comms/aura_messages.h"
#include "comms/remote_link.hxx"
#include "comms/logging.hxx"
#include "filters/nav_ekf15/aura_interface.hxx"
#include "filters/nav_ekf15_mag/aura_interface.hxx"
#include "include/globaldefs.h"
#include "init/globals.hxx"
#include "util/myprof.hxx"

#include "ground.hxx"
#include "wind.hxx"

#include "filter_mgr.hxx"

//
// Global variables
//

static double last_imu_time = 0.0;

// property nodes
static pyPropertyNode imu_node;
static pyPropertyNode gps_node;
static pyPropertyNode pos_node;
static pyPropertyNode orient_node;
static pyPropertyNode vel_node;
static pyPropertyNode pos_filter_node;
static pyPropertyNode pos_pressure_node;
static pyPropertyNode pos_combined_node;
static pyPropertyNode filter_node;
static pyPropertyNode filter_group_node;
static pyPropertyNode remote_link_node;
static pyPropertyNode status_node;
static vector<pyPropertyNode> sections;
static vector<pyPropertyNode> outputs;

static int remote_link_skip = 0;
static int logging_skip = 0;

void Filter_init() {
    // initialize imu property nodes
    imu_node = pyGetNode("/sensors/imu", true);
    gps_node = pyGetNode("/sensors/gps", true);
    pos_node = pyGetNode("/position", true);
    orient_node = pyGetNode("/orientation", true);
    vel_node = pyGetNode("/velocity", true);
    filter_node = pyGetNode("/filters/filter", true);
    filter_group_node = pyGetNode("/filters", true);
    pos_filter_node = pyGetNode("/position/filter", true);
    pos_pressure_node = pyGetNode("/position/pressure", true);
    pos_combined_node = pyGetNode("/position/combined", true);
    remote_link_node = pyGetNode("/comms/remote_link", true);
    status_node = pyGetNode("/status", true);

    pyPropertyNode remote_link_config = pyGetNode("/config/remote_link", true);
    pyPropertyNode logging_node = pyGetNode("/config/logging", true);
    remote_link_skip = remote_link_config.getLong("filter_skip");
    logging_skip = logging_node.getLong("filter_skip");

    // traverse configured modules
    pyPropertyNode group_node = pyGetNode("/config/filters", true);
    vector<string> children = group_node.getChildren();
    printf("Found %d filter sections\n", (int)children.size());
    for ( unsigned int i = 0; i < children.size(); i++ ) {
	pyPropertyNode section = group_node.getChild(children[i].c_str());
	sections.push_back(section);
	string module = section.getString("module");
	bool enabled = section.getBool("enable");
	if ( !enabled ) {
	    continue;
	}
	ostringstream output_path;
	output_path << "/filters/filter" << '[' << i << ']';
        pyPropertyNode output_node = pyGetNode(output_path.str(), true);
        outputs.push_back(output_node);
	printf("filter: %d = %s\n", i, module.c_str());
	if ( module == "null" ) {
	    // do nothing
	} else if ( module == "nav-ekf15" ) {
	    nav_ekf15_init( output_path.str(), &section );
	} else if ( module == "nav-ekf15-mag" ) {
	    nav_ekf15_mag_init( output_path.str(), &section );
	} else {
	    printf("Unknown filter = '%s' in config file\n",
		   module.c_str());
	}
    }

    // initialize ground estimator
    init_ground();
    
    // initialize wind estimator
    init_wind();
}


static void update_euler_rates() {
    double phi = orient_node.getDouble("roll_deg") * SGD_DEGREES_TO_RADIANS;
    double the = orient_node.getDouble("pitch_deg") * SGD_DEGREES_TO_RADIANS;

    // direct computation of euler rates given body rates and estimated
    // attitude (based on googled references):
    // http://www.princeton.edu/~stengel/MAE331Lecture9.pdf
    // http://www.mathworks.com/help/aeroblks/customvariablemass6dofeulerangles.html

    double p = imu_node.getDouble("p_rad_sec");
    double q = imu_node.getDouble("q_rad_sec");
    double r = imu_node.getDouble("r_rad_sec");

    if ( SGD_PI_2 - fabs(the) > 0.00001 ) {
	double phi_dot = p + q * sin(phi) * tan(the) + r * cos(phi) * tan(the);
	double the_dot = q * cos(phi) - r * sin(phi);
	double psi_dot = q * sin(phi) / cos(the) + r * cos(phi) / cos(the);
	orient_node.setDouble("phi_dot_rad_sec", phi_dot);
	orient_node.setDouble("the_dot_rad_sec", the_dot);
	orient_node.setDouble("psi_dot_rad_sec", psi_dot);
	/* printf("dt=%.3f q=%.3f q(ned)=%.3f phi(dot)=%.3f\n",
	   dt,imu_node.getDouble("q_rad_sec"), dq/dt, phi_dot);  */
	/* printf("%.3f %.3f %.3f %.3f\n",
	   cur_time,imu_node.getDouble("q_rad_sec"), dq/dt, the_dot); */
   }
}


static void publish_values() {
    orient_node.setDouble( "roll_deg", filter_node.getDouble("roll_deg") );
    orient_node.setDouble( "pitch_deg", filter_node.getDouble("pitch_deg") );
    orient_node.setDouble( "heading_deg", filter_node.getDouble("heading_deg") );
    pos_node.setDouble( "latitude_deg", filter_node.getDouble("latitude_deg") );
    pos_node.setDouble( "longitude_deg", filter_node.getDouble("longitude_deg") );
    pos_filter_node.setDouble("altitude_m", filter_node.getDouble("altitude_m"));
    pos_filter_node.setDouble("altitude_ft", filter_node.getDouble("altitude_ft"));
    vel_node.setDouble( "vn_ms", filter_node.getDouble("vn_ms") );
    vel_node.setDouble( "ve_ms", filter_node.getDouble("ve_ms") );
    vel_node.setDouble( "vd_ms", filter_node.getDouble("vd_ms") );
    filter_group_node.setDouble( "timestamp",
				 filter_node.getDouble("timestamp") );
    status_node.setString( "navigation",
			   filter_node.getString("navigation") );
    bool use_filter = true;
    bool use_gps = !use_filter;
    if ( use_filter ) {
	orient_node.setDouble( "groundtrack_deg",
			       filter_node.getDouble("groundtrack_deg") );
	vel_node.setDouble( "groundspeed_ms",
			    filter_node.getDouble("groundspeed_ms") );
    } else if ( use_gps ) {
	const double R2D = 57.295779513082323;
	double vn = gps_node.getDouble("vn_ms");
	double ve = gps_node.getDouble("ve_ms");
	filter_node.setDouble( "groundtrack_deg", 90 - atan2(vn, ve) * R2D );
	filter_node.setDouble( "groundspeed_ms", sqrt(vn*vn + ve*ve) );
    }
   
    vel_node.setDouble( "vertical_speed_fps",
			filter_node.getDouble("vertical_speed_fps") );
    
    // select official source (currently AGL is pressure based,
    // absolute ground alt is based on average gps/filter value at
    // startup, and MSL altitude is based on pressure altitude -
    // pressure error (pressure error computed as average difference
    // between gps altitude and pressure altitude over time)):
    //
    // 1. /position/pressure
    // 2. /position/filter
    // 3. /position/combined
    //official_alt_m_node->alias("/position/combined/altitude-true-m");
    //official_alt_ft_node->alias("/position/combined/altitude-true-ft");
    //official_agl_m_node->alias("/position/pressure/altitude-agl-m");
    //official_agl_ft_node->alias("/position/pressure/altitude-agl-ft");
    //official_ground_m_node->alias("/position/filter/altitude-ground-m");    

    // the following block favors the baro based altimeter, but can
    // suffer from cabin pressure change bias, temperature bias, or
    // other unexplained biases.
    // pos_node.setDouble( "altitude_m",
    //     		pos_combined_node.getDouble("altitude_true_m") );
    // pos_node.setDouble( "altitude_ft",
    //     		pos_combined_node.getDouble("altitude_true_ft") );
    // pos_node.setDouble( "altitude_agl_m",
    //     		pos_pressure_node.getDouble("altitude_agl_m") );
    // pos_node.setDouble( "altitude_agl_ft",
    //     		pos_pressure_node.getDouble("altitude_agl_ft") );
    // pos_node.setDouble( "altitude_ground_m",
    //     		pos_filter_node.getDouble("altitude_ground_m") );

    // the following block favor the filter based altitude which can
    // be adversely affected (significantly) by gps altitude errors.
    pos_node.setDouble( "altitude_m",
			pos_filter_node.getDouble("altitude_m") );
    pos_node.setDouble( "altitude_ft",
			pos_filter_node.getDouble("altitude_ft") );
    pos_node.setDouble( "altitude_agl_m",
			pos_filter_node.getDouble("altitude_agl_m") );
    pos_node.setDouble( "altitude_agl_ft",
			pos_filter_node.getDouble("altitude_agl_ft") );
    pos_node.setDouble( "altitude_ground_m",
			pos_filter_node.getDouble("altitude_ground_m") );
}

bool Filter_update() {
    filter_prof.start();

    double imu_time = imu_node.getDouble("timestamp");
    double imu_dt = imu_time - last_imu_time;
    bool fresh_filter_data = false;

    // sanity check (i.e. if system clock was changed by another process)
    if ( imu_dt > 1.0 ) { imu_dt = 0.01; }
    if ( imu_dt < 0.0 ) { imu_dt = 0.01; }

    static int remote_link_count = 0;
    static int logging_count = 0;
    
    // experimental: reinit all the ekf's upon request
    string command = filter_group_node.getString( "command" );
    bool do_reset = false;
    if ( command.length() ) {
        if ( command == "reset" ) {
            do_reset = true;
            filter_group_node.setString( "command_result",
                                         "success: " + command );
        } else {
            // unknown command
            filter_group_node.setString( "command_result",
                                         "unknown command: " + command );
        }
        filter_group_node.setString( "command", "" );
    }

    // traverse configured modules
    for ( unsigned int i = 0; i < sections.size(); i++ ) {
	string module = sections[i].getString("module");
	bool enabled = sections[i].getBool("enable");
	if ( !enabled ) {
	    continue;
	}
	if ( module == "null" ) {
	    // do nothing
	} else if ( module == "nav-ekf15" ) {
            if ( do_reset ) {
                nav_ekf15_reset();
            }
	    fresh_filter_data = nav_ekf15_update();
	} else if ( module == "nav-ekf15-mag" ) {
            if ( do_reset ) {
                nav_ekf15_mag_reset();
            }
	    fresh_filter_data = nav_ekf15_mag_update();
	}
	if ( fresh_filter_data ) {
	    if ( i == 0 ) {
		// only for primary filter
		update_euler_rates();
		update_ground(imu_dt);
		update_wind(imu_dt);
		publish_values();
	    }
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
            message_filter_v4_t nav;
            nav.index = i;
            nav.timestamp_sec = outputs[i].getDouble("timestamp");
            nav.latitude_deg = outputs[i].getDouble("latitude_deg");
            nav.longitude_deg = outputs[i].getDouble("longitude_deg");
            nav.altitude_m = outputs[i].getDouble("altitude_m");
            nav.vn_ms = outputs[i].getDouble("vn_ms");
            nav.ve_ms = outputs[i].getDouble("ve_ms");
            nav.vd_ms = outputs[i].getDouble("vd_ms");
            nav.roll_deg = outputs[i].getDouble("roll_deg");
            nav.pitch_deg = outputs[i].getDouble("pitch_deg");
            nav.yaw_deg = outputs[i].getDouble("heading_deg");
            nav.p_bias = outputs[i].getDouble("p_bias");
            nav.q_bias = outputs[i].getDouble("q_bias");
            nav.r_bias = outputs[i].getDouble("r_bias");
            nav.ax_bias = outputs[i].getDouble("ax_bias");
            nav.ay_bias = outputs[i].getDouble("ay_bias");
            nav.az_bias = outputs[i].getDouble("az_bias");
            nav.sequence_num = remote_link_node.getLong("sequence_num");
            nav.status = 0;
            nav.pack();
	    if ( send_remote_link ) {
		remote_link->send_message( nav.id, nav.payload, nav.len );
	    }
	    if ( send_logging ) {
		logging->log_message( nav.id, nav.payload, nav.len );
	    }
	}
    }

    filter_prof.stop();

    if ( fresh_filter_data ) {
        remote_link_count--;
        logging_count--;
    }
	     
    last_imu_time = imu_time;

#if 0
    //static SGPropertyNode *tp = fgGetNode("/sensors/imu/pitch-truth-deg", true);
    //static SGPropertyNode *ep = fgGetNode("/orientation/pitch-deg", true);
    printf("pitch error: %.2f (true = %.2f est = %.2f)\n", ep.getDouble() - tp.getDouble(), tp.getDouble(), ep.getDouble());
#endif

    return fresh_filter_data;
}


void Filter_close() {
    // traverse configured modules
    for ( unsigned int i = 0; i < sections.size(); i++ ) {
	string module = sections[i].getString("module");
	bool enabled = sections[i].getBool("enable");
	if ( !enabled ) {
	    continue;
	}
	if ( module == "null" ) {
	    // do nothing
	} else if ( module == "nav-ekf15" ) {
	    nav_ekf15_close();
	} else if ( module == "nav-ekf15-mag" ) {
	    nav_ekf15_mag_close();
	}
    }
}
