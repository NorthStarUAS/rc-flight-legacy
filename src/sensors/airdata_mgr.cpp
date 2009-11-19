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

#include "comms/logging.h"
#include "include/globaldefs.h"
#include "props/props.hxx"
#include "util/myprof.h"

#ifdef ENABLE_MNAV_SENSOR
#  include "filters/mnav/ahrs.h"
#  include "mnav.h"
#endif

#include "airdata_mgr.h"

//
// Global variables
//

static float Ps_filt = 0.0;
static float Pt_filt = 0.0;
static float true_alt_m = 0.0;
static float ground_alt_press = 0.0;
static float climb_filt = 0.0;
static float accel_filt = 0.0;

// air data property nodes
static SGPropertyNode *airdata_timestamp_node = NULL;
static SGPropertyNode *airdata_Ps_node = NULL;
static SGPropertyNode *airdata_Pt_node = NULL;
static SGPropertyNode *Ps_filt_node = NULL;
static SGPropertyNode *Pt_filt_node = NULL;
static SGPropertyNode *pressure_error_m_node = NULL;
static SGPropertyNode *true_alt_ft_node = NULL;
static SGPropertyNode *agl_alt_ft_node = NULL;
static SGPropertyNode *vert_fps_node = NULL;
static SGPropertyNode *forward_accel_node = NULL;
static SGPropertyNode *ground_alt_press_m_node = NULL;


void AirData_init() {
    // initialize air data property nodes
    airdata_timestamp_node = fgGetNode("/sensors/air-data/timestamp", true);
    airdata_Ps_node = fgGetNode("/sensors/air-data/Ps-m", true);
    airdata_Pt_node = fgGetNode("/sensors/air-data/Pt-ms", true);
    Ps_filt_node = fgGetNode("/position/altitude-pressure-m", true);
    Pt_filt_node = fgGetNode("/velocities/airspeed-kt", true);
    true_alt_ft_node = fgGetNode("/position/altitude-ft",true);
    agl_alt_ft_node = fgGetNode("/position/altitude-agl-ft", true);
    pressure_error_m_node
	= fgGetNode("/position/pressure-error-m", true);
    vert_fps_node
	= fgGetNode("/velocities/pressure-vertical-speed-fps",true);
    forward_accel_node = fgGetNode("/accelerations/airspeed-ktps",true);
    ground_alt_press_m_node
        = fgGetNode("/position/ground-altitude-pressure-m", true);

    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/sensors/airdata-group", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "air-data" ) {
	    string source = section->getChild("source")->getStringValue();
	    string basename = "/sensors/";
	    basename += section->getDisplayName();
	    printf("i = %d  name = %s source = %s %s\n",
		   i, name.c_str(), source.c_str(), basename.c_str());
	    if ( source == "null" ) {
		// do nothing
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


static void do_pressure_helpers() {
    static float Ps_filt_last = 0.0;
    static float Pt_filt_last = 0.0;
    static double t_last = 0.0;

    double ad_time = airdata_timestamp_node->getDoubleValue();
    double Ps = airdata_Ps_node->getDoubleValue();
    double Pt = airdata_Pt_node->getDoubleValue();

    // Do a simple first order low pass filter to reduce noise
    Ps_filt = 0.97 * Ps_filt + 0.03 * Ps;
    Pt_filt = 0.97 * Pt_filt + 0.03 * Pt;

    // best guess at true altitude
    true_alt_m = Ps_filt + pressure_error_m_node->getDoubleValue();

    // compute rate of climb based on pressure altitude change
    float climb = (Ps_filt - Ps_filt_last) / (ad_time - t_last);
    Ps_filt_last = Ps_filt;
    climb_filt = 0.97 * climb_filt + 0.03 * climb;

    // compute a forward acceleration value based on pitot speed
    // change
    float accel = (Pt_filt - Pt_filt_last) / (ad_time - t_last);
    Pt_filt_last = Pt_filt;
    accel_filt = 0.97 * accel_filt + 0.03 * accel;

    // determine ground reference altitude.  Average pressure
    // altitude over first 30 seconds unit is powered on.  This
    // assumes that system clock time starts counting at zero.
    // Watch out if we ever find a way to set the system clock to
    // real time.
    static bool first_time = true;
    if ( first_time ) { 
	ground_alt_press = Ps;
	first_time = false;
    }
    if ( ad_time < 30.0 ) {
	float dt = ad_time - t_last;
	float elapsed = ad_time - dt;
	ground_alt_press
	    = (elapsed * ground_alt_press + dt * Ps)
	    / ad_time;
	ground_alt_press_m_node->setDoubleValue( ground_alt_press );
    }

    t_last = ad_time;

    // printf("Ps = %.1f nav = %.1f bld = %.1f vsi = %.2f\n",
    //        Ps_filt, navpacket.alt, true_alt_m, climb_filt);
}


bool AirData_update() {
    air_prof.start();

    bool fresh_data = false;

    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/sensors/airdata-group", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "air-data" ) {
	    string source = section->getChild("source")->getStringValue();
	    string basename = "/sensors/";
	    basename += section->getDisplayName();
	    // printf("i = %d  name = %s source = %s %s\n",
	    //	   i, name.c_str(), source.c_str(), basename.c_str());
	    if ( source == "null" ) {
		// do nothing
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
	do_pressure_helpers();

	// publish values to property tree
	Ps_filt_node->setDoubleValue( Ps_filt );
	Pt_filt_node->setDoubleValue( Pt_filt * SG_MPS_TO_KT );
	true_alt_ft_node->setDoubleValue( true_alt_m * SG_METER_TO_FEET );
	agl_alt_ft_node->setDoubleValue( (Ps_filt - ground_alt_press)
					* SG_METER_TO_FEET );
	vert_fps_node->setDoubleValue( climb_filt * SG_METER_TO_FEET );
	forward_accel_node->setDoubleValue( accel_filt * SG_MPS_TO_KT );
    }

    air_prof.stop();

    return fresh_data;
}


void AirData_close() {
    // traverse configured modules
    SGPropertyNode *toplevel = fgGetNode("/config/sensors/airdata-group", true);
    for ( int i = 0; i < toplevel->nChildren(); ++i ) {
	SGPropertyNode *section = toplevel->getChild(i);
	string name = section->getName();
	if ( name == "airdata" ) {
	    string source = section->getChild("source")->getStringValue();
	    string basename = "/sensors/";
	    basename += section->getDisplayName();
	    // printf("i = %d  name = %s source = %s %s\n",
	    //	   i, name.c_str(), source.c_str(), basename.c_str());
	    if ( source == "mnav" ) {
		// nop
	    } else {
		printf("Unknown air data source = '%s' in config file\n",
		       source.c_str());
	    }
	}
    }
}
