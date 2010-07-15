/**
 * \file: airdata_mgr.cpp
 *
 * Front end management interface for reading air data.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@gmail.com
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

#ifdef ENABLE_MNAV_SENSOR
#  include "filters/mnav/ahrs.h"
#  include "mnav.h"
#endif

#include "actuators/ardusensor.hxx"
#include "imu_fgfs.hxx"

#include "airdata_mgr.h"

//
// Global variables
//

static float altitude_filt = 0.0;
static float airspeed_filt = 0.0;
static float true_alt_m = 0.0;
static float ground_alt_press = 0.0;
static float climb_filt = 0.0;
static float accel_filt = 0.0;
static float Ps_filt_err = 0.0;

// air data property nodes
static SGPropertyNode *airdata_timestamp_node = NULL;
static SGPropertyNode *airdata_altitude_node = NULL;
static SGPropertyNode *airdata_airspeed_node = NULL;

// input property nodes
static SGPropertyNode *filter_timestamp_node = NULL;
static SGPropertyNode *filter_alt_node = NULL;

// output property nodes
static SGPropertyNode *altitude_filt_node = NULL;
static SGPropertyNode *airspeed_filt_node = NULL;
static SGPropertyNode *pressure_error_m_node = NULL;
static SGPropertyNode *true_alt_ft_node = NULL;
static SGPropertyNode *agl_alt_ft_node = NULL;
static SGPropertyNode *vert_fps_node = NULL;
static SGPropertyNode *forward_accel_node = NULL;
static SGPropertyNode *ground_alt_press_m_node = NULL;

// comm property nodes
static SGPropertyNode *airdata_console_skip = NULL;
static SGPropertyNode *airdata_logging_skip = NULL;


void AirData_init() {
    // initialize air data property nodes
    airdata_timestamp_node = fgGetNode("/sensors/air-data/time-stamp", true);
    airdata_altitude_node = fgGetNode("/sensors/air-data/altitude-m", true);
    airdata_airspeed_node = fgGetNode("/sensors/air-data/airspeed-kt", true);

    // input property nodes
    filter_timestamp_node = fgGetNode("/filters/filter/time-stamp", true);
    filter_alt_node = fgGetNode("/position/altitude-m", true);

    // filtered/computed output property nodes
    altitude_filt_node = fgGetNode("/position/altitude-pressure-m", true);
    airspeed_filt_node = fgGetNode("/velocity/airspeed-kt", true);

    true_alt_ft_node = fgGetNode("/position/altitude-true-combined-ft",true);
    agl_alt_ft_node = fgGetNode("/position/altitude-pressure-agl-ft", true);

    pressure_error_m_node
	= fgGetNode("/position/pressure-error-m", true);
    vert_fps_node
	= fgGetNode("/velocity/pressure-vertical-speed-fps",true);
    forward_accel_node = fgGetNode("/acceleration/airspeed-ktps",true);
    ground_alt_press_m_node
        = fgGetNode("/position/ground-altitude-pressure-m", true);

    // initialize comm nodes
    airdata_console_skip = fgGetNode("/config/console/airdata-skip", true);
    airdata_logging_skip = fgGetNode("/config/logging/airdata-skip", true);

    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/sensors/air-data-group", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "air-data" ) {
	    string source = section->getChild("source", 0, true)->getStringValue();
	    string basename = "/sensors/";
	    basename += section->getDisplayName();
	    printf("i = %d  name = %s source = %s %s\n",
		   i, name.c_str(), source.c_str(), basename.c_str());
	    if ( source == "null" ) {
		// do nothing
	    } else if ( source == "ardusensor" ) {
		ardusensor_airdata_init( basename );
	    } else if ( source == "fgfs" ) {
		fgfs_airdata_init( basename );
#ifdef ENABLE_MNAV_SENSOR
	    } else if ( source == "mnav" ) {
		mnav_airdata_init( basename );
#endif // ENABLE_MNAV_SENSOR
	    } else {
		printf("Unknown air data source = '%s' in config file\n",
		       source.c_str());
	    }
	}
    }
}


static void update_pressure_helpers() {
    static float altitude_filt_last = 0.0;
    static float airspeed_filt_last = 0.0;
    static double last_time = 0.0;
    double cur_time = airdata_timestamp_node->getDoubleValue();
    static double start_time = cur_time;

    double dt = cur_time - last_time;
    if ( dt > 1.0 ) {
	dt = 1.0;		// keep dt smallish
    }

    if ( dt < 0.000001 ) {
	return; 		// do nothing if dt zero
    }

    float Ps = airdata_altitude_node->getFloatValue();
    float Pt = airdata_airspeed_node->getFloatValue();
    float filter_alt_m = filter_alt_node->getFloatValue();

    // Do a simple first order (time based) low pass filter to reduce noise
    float time_factor = 0.5; // length of time (sec) to low pass
			     // filter the input over.  A time value
			     // of zero will result in the filter
			     // output being equal to the raw input at
			     // each time step.
    float weight_factor;
    if ( time_factor > 0.0 ) {
	weight_factor = dt / time_factor;
    } else {
	weight_factor = 1.0;
    }
    // The greater the weight, the noisier the filter, but the faster
    // it converges.  Must be > 0.0 or value will never converge.  Max
    // weight is 1.0 which means we just use the raw input value with
    // no filetering.
    if ( weight_factor < 0.001 ) {
	weight_factor = 0.001;
    }
    if ( weight_factor > 1.0 ) {
	weight_factor = 1.0;
    }
    altitude_filt = (1.0 - weight_factor) * altitude_filt + weight_factor * Ps;
    airspeed_filt = (1.0 - weight_factor) * airspeed_filt + weight_factor * Pt;

    // compute a filtered error difference between gps altitude
    // and pressure altitude.  (at 4hz update rate this averages
    // the error over about 40 minutes)
    double elapsed_time = cur_time - start_time;
    if ( elapsed_time > 1800 ) {
	elapsed_time = 1800;	// 30 minutes
    }
    static bool first_time = true;
    if ( first_time ) {
	if ( filter_timestamp_node->getDoubleValue() > 1.0 ) {
	    first_time = false;
	    Ps_filt_err = filter_alt_m - Ps;
	}
    } else {
	if ( elapsed_time >= dt && elapsed_time >= 0.001 ) {
	    float alt_err = filter_alt_m - Ps;
	    Ps_filt_err = ((elapsed_time - dt) * Ps_filt_err + dt * alt_err)
		/ elapsed_time;
	    // printf("cnt = %.0f err = %.2f\n", Ps_count, Ps_filt_err);
	}

	// best guess at true altitude
	true_alt_m = altitude_filt + Ps_filt_err;
    }

    // compute rate of climb based on pressure altitude change
    float climb = (altitude_filt - altitude_filt_last) / dt;
    altitude_filt_last = altitude_filt;
    climb_filt = 0.97 * climb_filt + 0.03 * climb;

    // compute a forward acceleration value based on pitot speed
    // change
    float accel = (airspeed_filt - airspeed_filt_last) / dt;
    airspeed_filt_last = airspeed_filt;
    accel_filt = 0.97 * accel_filt + 0.03 * accel;

    // determine ground reference altitude.  Average filter altitude
    // over first 30 seconds the filter becomes active.
    static float ground_alt_filter = Ps;

    if ( elapsed_time >= dt && elapsed_time >= 0.001 && elapsed_time <= 30.0 ) {
	ground_alt_filter
	    = ((elapsed_time - dt) * ground_alt_filter + dt * Ps)
	    / elapsed_time;
	ground_alt_press_m_node->setDoubleValue( ground_alt_filter );
    }

    last_time = cur_time;

    // publish values to property tree
    pressure_error_m_node->setDoubleValue( Ps_filt_err );
    altitude_filt_node->setDoubleValue( altitude_filt );
    airspeed_filt_node->setDoubleValue( airspeed_filt );
    true_alt_ft_node->setDoubleValue( true_alt_m * SG_METER_TO_FEET );
    agl_alt_ft_node->setDoubleValue( (altitude_filt - ground_alt_filter)
				     * SG_METER_TO_FEET );
    vert_fps_node->setDoubleValue( climb_filt * SG_METER_TO_FEET );
    forward_accel_node->setDoubleValue( accel_filt * SG_MPS_TO_KT );

    // printf("Ps = %.1f nav = %.1f bld = %.1f vsi = %.2f\n",
    //        altitude_filt, navpacket.alt, true_alt_m, climb_filt);
}


bool AirData_update() {
    air_prof.start();

    bool fresh_data = false;

    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/sensors/air-data-group", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "air-data" ) {
	    string source = section->getChild("source", 0, true)->getStringValue();
	    string basename = "/sensors/";
	    basename += section->getDisplayName();
	    // printf("i = %d  name = %s source = %s %s\n",
	    //	   i, name.c_str(), source.c_str(), basename.c_str());
	    if ( source == "null" ) {
		// do nothing
	    } else if ( source == "ardusensor" ) {
		fresh_data = ardusensor_airdata_update();
	    } else if ( source == "fgfs" ) {
		fresh_data = fgfs_airdata_update();
#ifdef ENABLE_MNAV_SENSOR
	    } else if ( source == "mnav" ) {
		fresh_data = mnav_get_airdata();
#endif // ENABLE_MNAV_SENSOR
	    } else {
		printf("Unknown air data source = '%s' in config file\n",
		       source.c_str());
	    }
	}
    }

    if ( fresh_data ) {
	update_pressure_helpers();

	if ( console_link_on || log_to_file ) {
	    uint8_t buf[256];
	    int size = packetizer->packetize_airdata( buf );

	    if ( console_link_on ) {
		// printf("sending filter packet\n");
		console_link_airdata( buf, size,
				      airdata_console_skip->getIntValue() );
	    }

	    if ( log_to_file ) {
		log_airdata( buf, size, airdata_logging_skip->getIntValue() );
	    }
	}
    }

    air_prof.stop();

    return fresh_data;
}


void AirData_close() {
    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/sensors/air-data-group", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "airdata" ) {
	    string source = section->getChild("source", 0, true)->getStringValue();
	    string basename = "/sensors/";
	    basename += section->getDisplayName();
	    // printf("i = %d  name = %s source = %s %s\n",
	    //	   i, name.c_str(), source.c_str(), basename.c_str());
	    if ( source == "null" ) {
		// do nothing
	    } else if ( source == "fgfs" ) {
		// nop
	    } else if ( source == "ardusensor" ) {
		// nop
#ifdef ENABLE_MNAV_SENSOR
	    } else if ( source == "mnav" ) {
		// nop
#endif
	    } else {
		printf("Unknown air data source = '%s' in config file\n",
		       source.c_str());
	    }
	}
    }
}
