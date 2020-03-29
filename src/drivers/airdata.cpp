/**
 * \file: airdata_mgr.cpp
 *
 * Front end management interface for reading air data.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@flightgear.org
 *
 */


#include <math.h>
#include <stdio.h>
#include <string.h>

#include <string>
using std::string;

#include "include/globaldefs.h"
#include "util/myprof.h"

#include "airdata.h"

void airdata_helper_t::init() {
    airdata_node = pyGetNode("/sensors/airdata", true);
    sensors_node = pyGetNode("/sensors", true);
    pos_filter_node = pyGetNode("/position/filter", true);
    pos_pressure_node = pyGetNode("/position/pressure", true);
    pos_combined_node = pyGetNode("/position/combined", true);
    status_node = pyGetNode("/status", true);
    task_node = pyGetNode("/task", true);
    vel_node = pyGetNode("/velocity", true);
    wind_node = pyGetNode("/filters/wind", true);
}

// 1. ground altitude, 2. error between pressure altitude and gps altitude
void airdata_helper_t::update() {
    airdata_prof.start();

    double cur_time = airdata_node.getDouble("timestamp");

    double dt = cur_time - last_time;
    if ( dt > 1.0 ) {
	dt = 1.0;		// keep dt smallish
    }

    if ( dt < 0.000001 ) {
	return; 		// do nothing if dt zero
    }

    //
    // 1. Compute altitude from airdata pressure sensor
    //

    // Forumula taken from:
    //   http://keisan.casio.com/exec/system/1224585971
    // or possibly:
    //   http://keisan.casio.com/has10/SpecExec.cgi?path=06000000%2eScience%2f02100100%2eEarth%20science%2f12000300%2eAltitude%20from%20atmospheric%20pressure%2fdefault%2exml&charset=utf-8
    //
    // h = (((P0/P)^(1/5.257) - 1) * (T+273.15)) / 0.0065
    // T = h*0.0065 / ((P0/P)^(1/5.257) - 1) - 273.15

    const double P0 = 1013.25;	// standard sea level pressure
    double P = airdata_node.getDouble("pressure_mbar"); // sensed pressure
    if ( P < 0.1 ) {
        P = P0;
    }

    // The APM temp sensor is highly biased by board temp and cabin
    // interior temperature (not OAT), so it really makes more sense
    // to just pick a fixed value here so our relative altitude does
    // not drift as our airplane temp changes.  We have a later system
    // that estimates the error between gps altitude and pressure
    // altitude.
    const double T = 15.0;	// standard temp

    // Compute altitude on a standard day
    double tmp1 = pow((P0/P), 1.0/5.257) - 1.0;
    double alt_m = (tmp1 * (T + 273.15)) / 0.0065;
    pos_pressure_node.setDouble( "altitude_m", alt_m );

    //
    // 2. Filter/Smooth Altitude and airspeed to reduce noise
    //

    float Pt = airdata_node.getDouble("airspeed_kt");
    float Ps = alt_m; /* pressure_alt_node.getDouble(); */
    float filter_alt_m = pos_filter_node.getDouble("altitude_m");

    if ( !airdata_calibrated ) {
	airdata_calibrated = true;
	airspeed_filt.init( Pt );
	pressure_alt_filt.init( Ps );
	ground_alt_filt.init( Ps );
	climb_filt.init( 0.0 );
    }
    
    airspeed_filt.update( Pt, dt );
    pressure_alt_filt.update( Ps, dt );
    if ( ! task_node.getBool("is_airborne") ) {
	// ground reference altitude averaged current altitude over
	// first 30 seconds while on the ground
	ground_alt_filt.update( Ps, dt );
    }

    // publish values
    vel_node.setDouble( "airspeed_kt", Pt /* raw */ );
    vel_node.setDouble( "airspeed_smoothed_kt", airspeed_filt.get_value() );
    pos_pressure_node.setDouble( "altitude_smoothed_m", pressure_alt_filt.get_value() );
    pos_pressure_node.setDouble( "altitude_ground_m", ground_alt_filt.get_value() );

    //
    // 3. Compute a filtered error difference between gps altitude and
    //    pressure altitude.
    //

    if ( !alt_error_calibrated ) {
	if ( status_node.getString("navigation") == "ok" ) {
	    alt_error_calibrated = true;
	    Ps_filt_err.init( filter_alt_m - Ps );
	}
    } else {
	Ps_filt_err.update( filter_alt_m - Ps, dt );

	// best guess at true altitude
	true_alt_m = pressure_alt_filt.get_value() + Ps_filt_err.get_value();
    }

    // true altitude estimate - filter ground average is our best
    // estimate of true agl if altitude has not changed recently.
    double true_agl_m = true_alt_m - pos_filter_node.getDouble("altitude_ground_m");

    //
    // 4. Compute some other stuff
    // 

    // compute rate of climb based on pressure altitude change
    float climb = (pressure_alt_filt.get_value() - pressure_alt_filt_last) / dt;
    pressure_alt_filt_last = pressure_alt_filt.get_value();
    // sanity check, discard fabs(values) > 16 mps (3276.7 fpm)
    if ( climb > 16.0 ) {
	climb = 0.0;
    } else if ( climb < -16.0 ) {
	climb = 0.0;
    }
    climb_filt.update( climb, dt );
    last_time = cur_time;

    // publish values to property tree
    pos_pressure_node.setDouble( "pressure_error_m", Ps_filt_err.get_value() );
    pos_combined_node.setDouble( "altitude_true_m", true_alt_m );
    pos_combined_node.setDouble( "altitude_true_ft",
				 true_alt_m * SG_METER_TO_FEET );
    pos_combined_node.setDouble( "altitude_agl_m", true_agl_m );
    pos_combined_node.setDouble( "altitude_agl_ft",
				 true_agl_m * SG_METER_TO_FEET );
    pos_pressure_node.setDouble( "altitude_agl_m",
				 pressure_alt_filt.get_value()
				 - ground_alt_filt.get_value() );
    pos_pressure_node.setDouble( "altitude_agl_ft",
				 (pressure_alt_filt.get_value()
				  - ground_alt_filt.get_value() )
				 * SG_METER_TO_FEET );
    vel_node.setDouble( "pressure_vertical_speed_fps",
			climb_filt.get_value() * SG_METER_TO_FEET );

    // printf("Ps = %.1f nav = %.1f bld = %.1f vsi = %.2f\n",
    //        pressure_alt_filt, navpacket.alt, true_alt_m, climb_filt.get_value());

#if 0
    // experimental section ... try to estimate thermal activity ...
    static SGPropertyNode *throttle = pyGetNode("/controls/engine/throttle", true);
    static double sum_x = 0.0;
    static double sum_y = 0.0;
    static double sum_x2 = 0.0;
    static double sum_y2 = 0.0;
    static double sum_xy = 0.0;

    double x = throttle.getDouble();
    double y = climb_filt.get_value() * SG_METER_TO_FEET * 60.0; // fpm
    double n = 6000.0;		// 100hz * 60 sec
    double nfact = (n-1.0)/n;
    sum_x = sum_x*nfact + x;
    sum_y = sum_y*nfact + y;
    //printf("x=%.2f y=%.0f sum_x=%.1f sum_y=%.0f\n", x, y, sum_x, sum_y);
    sum_x2 = sum_x2*nfact + x*x;
    sum_y2 = sum_y2*nfact + y*y;
    sum_xy = sum_xy*nfact + x*y;

    double a1 = (n*sum_xy - sum_x*sum_y) / (n*sum_x2 - sum_x*sum_x);
    double a0 = (sum_y - a1*sum_x) / n;
    printf("y = %.2f + %.2f * x\n", a0, a1);
#endif

    airdata_prof.stop();
}

// global shared instance
airdata_helper_t airdata_helper;
