/**
 * \file: filter_mgr.cpp
 *
 * Front end management interface for executing the available filter codes.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson <at> gmail <dot> com
 *
 * $Id: adns_mgr.cpp,v 1.6 2009/05/15 17:04:56 curt Exp $
 */


#include "python/pyprops.hxx"

#include <stdio.h>
#include <string>
#include <sstream>
using std::string;
using std::ostringstream;

#include "comms/remote_link.hxx"
#include "comms/logging.hxx"
#include "filters/curt/adns_curt.hxx"
#include "filters/umngnss_euler/umngnss_euler.hxx"
#include "filters/umngnss_quat/umngnss_quat.hxx"
#include "include/globaldefs.h"
#include "init/globals.hxx"
#include "util/lowpass.hxx"
#include "util/myprof.h"

#include "filter_mgr.hxx"

//
// Global variables
//

static double last_imu_time = 0.0;

// property nodes
static pyPropertyNode imu_node;
static pyPropertyNode pos_node;
static pyPropertyNode orient_node;
static pyPropertyNode vel_node;
static pyPropertyNode pos_filter_node;
static pyPropertyNode pos_pressure_node;
static pyPropertyNode pos_combined_node;
static pyPropertyNode filter_node;
static pyPropertyNode filter_group_node;
static pyPropertyNode task_node;
static pyPropertyNode airdata_node;
static pyPropertyNode wind_node;
static pyPropertyNode status_node;
static vector<pyPropertyNode> sections;

// initial values are the 'time factor'
static LowPassFilter ground_alt_filt( 30.0 );
static bool ground_alt_calibrated = false;

static int remote_link_skip = 0;
static int logging_skip = 0;

void Filter_init() {
    // initialize imu property nodes
    imu_node = pyGetNode("/sensors/imu", true);
    pos_node = pyGetNode("/position", true);
    orient_node = pyGetNode("/orientation", true);
    vel_node = pyGetNode("/velocity", true);
    filter_node = pyGetNode("/filters/filter", true);
    filter_group_node = pyGetNode("/filters", true);
    pos_filter_node = pyGetNode("/position/filter", true);
    pos_pressure_node = pyGetNode("/position/pressure", true);
    pos_combined_node = pyGetNode("/position/combined", true);
    task_node = pyGetNode("/task", true);
    airdata_node = pyGetNode("/sensors/airdata", true);
    wind_node = pyGetNode("/filters/wind-est", true);
    status_node = pyGetNode("/status", true);

    wind_node.setDouble( "pitot_scale_factor", 1.0 );
    
    pyPropertyNode remote_link_node = pyGetNode("/config/remote_link", true);
    pyPropertyNode logging_node = pyGetNode("/config/logging", true);
    remote_link_skip = remote_link_node.getDouble("filter_skip");
    logging_skip = remote_link_node.getDouble("filter_skip");

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
	printf("filter: %d = %s\n", i, module.c_str());
	if ( module == "curt" ) {
	    curt_adns_init( output_path.str(), &section );
	} else if ( module == "umn_euler" ) {
	    umngnss_euler_init( output_path.str(), &section );
	} else if ( module == "umn_quat" ) {
	    umngnss_quat_init( output_path.str(), &section );
	} else {
	    printf("Unknown filter = '%s' in config file\n",
		   module.c_str());
	}
    }
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


static void update_ground() {
    static double last_time = 0.0;

    double cur_time = filter_node.getDouble("timestamp");
    double dt = cur_time - last_time;
    if ( dt > 1.0 ) {
	dt = 1.0;		// keep dt smallish
    }

    // determine ground reference altitude.  Average filter altitude
    // over the most recent 30 seconds that we are !is_airborne
    if ( !ground_alt_calibrated ) {
	ground_alt_calibrated = true;
	ground_alt_filt.init( filter_node.getDouble("altitude_m") );
    }

    if ( ! task_node.getBool("is_airborne") ) {
	// ground reference altitude averaged current altitude over
	// first 30 seconds while on the ground
	ground_alt_filt.update( filter_node.getDouble("altitude_m"), dt );
	pos_filter_node.setDouble( "altitude_ground_m",
				   ground_alt_filt.get_value() );
    }

    float agl_m = filter_node.getDouble( "altitude_m" )
	- ground_alt_filt.get_value();
    pos_filter_node.setDouble( "altitude_agl_m", agl_m );
    pos_filter_node.setDouble( "altitude_agl_ft", agl_m * SG_METER_TO_FEET );

    last_time = cur_time;
}


// onboard wind estimate (requires airspeed, true heading, and ground
// velocity fector)
static void update_wind() {
    // Estimate wind direction and speed based on ground track speed
    // versus aircraft heading and indicated airspeed.
    static double pitot_scale_filt = 1.0;

    double airspeed_kt = airdata_node.getDouble("airspeed_kt");
    if ( airspeed_kt < 15.0 ) {
	// indicated airspeed < 15 kts (hopefully) indicating we are
	// not flying and thus the assumptions the following code is
	// based on do not yet apply so we should exit now.  We are
	// assuming that we won't see > 15 kts sitting still on the
	// ground and that our stall speed is above 15 kts.  Is there
	// a more reliable way to determine if we are "flying"
	// vs. "not flying"?

	return;
    }

    double psi = SGD_PI_2
	- orient_node.getDouble("heading_deg") * SG_DEGREES_TO_RADIANS;
    double ue = cos(psi) * (airspeed_kt * pitot_scale_filt * SG_KT_TO_MPS);
    double un = sin(psi) * (airspeed_kt * pitot_scale_filt * SG_KT_TO_MPS);
    double we = ue - filter_node.getDouble("ve_ms");
    double wn = un - filter_node.getDouble("vn_ms");

    static double filt_we = 0.0, filt_wn = 0.0;
    filt_we = 0.9998 * filt_we + 0.0002 * we;
    filt_wn = 0.9998 * filt_wn + 0.0002 * wn;

    double wind_deg = 90 - atan2( filt_wn, filt_we ) * SGD_RADIANS_TO_DEGREES;
    if ( wind_deg < 0 ) { wind_deg += 360.0; }
    double wind_speed_kt = sqrt( filt_we*filt_we + filt_wn*filt_wn ) * SG_MPS_TO_KT;

    wind_node.setDouble( "wind_speed_kt", wind_speed_kt );
    wind_node.setDouble( "wind_dir_deg", wind_deg );
    wind_node.setDouble( "wind_east_mps", filt_we );
    wind_node.setDouble( "wind_north_mps", filt_wn );

    // estimate pitot tube bias
    double true_e = filt_we + filter_node.getDouble("ve_ms");
    double true_n = filt_wn + filter_node.getDouble("vn_ms");

    double true_deg = 90 - atan2( true_n, true_e ) * SGD_RADIANS_TO_DEGREES;
    if ( true_deg < 0 ) { true_deg += 360.0; }
    double true_speed_kt = sqrt( true_e*true_e + true_n*true_n ) * SG_MPS_TO_KT;

    wind_node.setDouble( "true_airspeed_kt", true_speed_kt );
    wind_node.setDouble( "true_heading_deg", true_deg );
    wind_node.setDouble( "true-airspeed-east-mps", true_e );
    wind_node.setDouble( "true_airspeed_north_mps", true_n );

    double pitot_scale = 1.0;
    if ( airspeed_kt > 1.0 ) {
	pitot_scale = true_speed_kt / airspeed_kt;
	// don't let the scale factor exceed some reasonable limits
	if ( pitot_scale < 0.75 ) { pitot_scale = 0.75;	}
	if ( pitot_scale > 1.25 ) { pitot_scale = 1.25; }
    }

    pitot_scale_filt = 0.9995 * pitot_scale_filt + 0.0005 * pitot_scale;
    wind_node.setDouble( "pitot_scale_factor", pitot_scale_filt );

    // if ( display_on ) {
    //   printf("true: %.2f kt  %.1f deg (scale = %.4f)\n", true_speed_kt, true_deg, pitot_scale_filt);
    // }
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
    orient_node.setDouble( "groundtrack_deg",
			   filter_node.getDouble("groundtrack_deg") );
    vel_node.setDouble( "groundspeed_ms",
			filter_node.getDouble("groundspeed_ms") );
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

    pos_node.setDouble( "altitude_m",
			pos_combined_node.getDouble("altitude_true_m") );
    pos_node.setDouble( "altitude_ft",
			pos_combined_node.getDouble("altitude_true_ft") );
    pos_node.setDouble( "altitude_agl_m",
			pos_pressure_node.getDouble("altitude_agl_m") );
    pos_node.setDouble( "altitude_agl_ft",
			pos_pressure_node.getDouble("altitude_agl_ft") );
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

    static int remote_link_count = remote_link_random( remote_link_skip );
    static int logging_count = remote_link_random( logging_skip );

    // traverse configured modules
    for ( unsigned int i = 0; i < sections.size(); i++ ) {
	string module = sections[i].getString("module");
	bool enabled = sections[i].getBool("enable");
	if ( !enabled ) {
	    continue;
	}
	if ( module == "null" ) {
	    // do nothing
	} else if ( module == "curt" ) {
	    fresh_filter_data = curt_adns_update( imu_dt );
	} else if ( module == "umn_euler" ) {
	    fresh_filter_data = umngnss_euler_update();
	} else if ( module == "umn_quat" ) {
	    fresh_filter_data = umngnss_quat_update();
	}
    }

    filter_prof.stop();

    if ( fresh_filter_data ) {
	update_euler_rates();
	update_ground();
	update_wind();
	publish_values();

	bool send_remote_link = false;
	if ( remote_link_on ) {
	    remote_link_count--;
	    if ( remote_link_count < 0 ) {
		send_remote_link = true;
		remote_link_count = remote_link_skip;
	    }
	}
	
	bool send_logging = false;
	if ( log_to_file ) {
	    logging_count--;
	    if ( logging_count < 0 ) {
		send_logging = true;
		logging_count = logging_skip;
	    }
	}
	
	if ( send_remote_link || send_logging ) {
	    uint8_t buf[256];
	    int size = packetizer->packetize_filter( buf );
	    if ( send_remote_link ) {
		remote_link_filter( buf, size );
	    }
	    if ( send_logging ) {
		log_filter( buf, size );
	    }
	}
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
	} else if ( module == "umn_euler" ) {
	    umngnss_euler_close();
	} else if ( module == "umn_quat" ) {
	    umngnss_quat_close();
	}
    }
}
