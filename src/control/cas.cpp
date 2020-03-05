// cas.cpp - Command Augmentation System (aka fly-by-wire)
//
// Written by Curtis Olson, started September 2010.
//
// Copyright (C) 2010  Curtis L. Olson - curtolson@flightgear.org
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Library General Public
// License as published by the Free Software Foundation; either
// version 2 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Library General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//


#include <pyprops.h>

#include <math.h>		// fabs()
#include <stdio.h>

#include "comms/display.h"
#include "util/timing.h"

#include "cas.h"


UGCAS cas;			// global cas object


UGCAS::UGCAS() :
    cas_mode( PassThrough )
{
}

UGCAS::~UGCAS() {
}


// bind property nodes
void UGCAS::bind() {
    config_node = pyGetNode("/config", true);
    pilot_node = pyGetNode("/sensors/pilot_input", true);
    cas_aileron_node = pyGetNode("/config/cas/aileron", true);
    cas_elevator_node = pyGetNode("/config/cas/elevator", true);
    targets_node = pyGetNode("/autopilot/targets", true);
    flight_node = pyGetNode("/controls/flight", true);
    engine_node = pyGetNode("/controls/engine", true);
}

// initialize CAS system
void UGCAS::init() {
    bind();
}

void UGCAS::update() {
    double current_time = get_Time();
    double dt = current_time - last_time;
    last_time = current_time;

    if ( !targets_node.getBool("master_switch") ) {
	// fcs master switch off, exit
	return;
    }

    // variables used to compute control center points and dead zone
    static bool stats_ready = false;
    static double start_time  = -1.0;
    static int count = 0;
    static double ail_sum = 0.0, elev_sum = 0.0;
    static double ail_center = 0.0, elev_center = 0.0;
    static double ail_max = -1.0, ail_min = 1.0;
    static double elev_max = -1.0, elev_min = 1.0;
    static double ail_dz = 0.0, elev_dz = 0.0;

    static double ail_pos_span = 1.0;
    static double ail_neg_span = 1.0;
    static double elev_pos_span = 1.0;
    static double elev_neg_span = 1.0;

    double aileron = pilot_node.getDouble("aileron");
    double elevator = pilot_node.getDouble("elevator");

    if ( ! stats_ready ) {
	if ( start_time < 0.0 ) {
	    start_time = current_time;
	}

	if ( current_time - start_time < 15.0 ) {
	    ail_sum += aileron;
	    elev_sum += elevator;
	    count++;

	    if ( aileron < ail_min ) { ail_min = aileron; }
	    if ( aileron > ail_max ) { ail_max = aileron; }
	    if ( elevator < elev_min ) { elev_min = elevator; }
	    if ( elevator > elev_max ) { elev_max = elevator; }

	    ail_center = ail_sum / (double)count;
	    elev_center = elev_sum / (double)count;

	    ail_dz = (ail_max - ail_min) / 2.0;
	    elev_dz = (elev_max - elev_min) / 2.0;
	} else {
	    stats_ready = true;
	    if ( display_on ) {
		printf("center ail = %.3f dead zone = %.3f\n",
		       ail_center, ail_dz);
		printf("center elev = %.3f dead zone = %.3f\n",
		       elev_center, elev_dz );
	    }
	}
    } else {
	ail_max = cas_aileron_node.getDouble("max");
	ail_min = cas_aileron_node.getDouble("min");
	ail_center = cas_aileron_node.getDouble("center");
	ail_dz = cas_aileron_node.getDouble("dead_zone");
	ail_pos_span = ail_max - (ail_center + ail_dz);
	ail_neg_span = (ail_center - ail_dz) - ail_min;

	elev_max = cas_elevator_node.getDouble("max");
	elev_min = cas_elevator_node.getDouble("min");
	elev_center = cas_elevator_node.getDouble("center");
	elev_dz = cas_elevator_node.getDouble("dead_zone");
	elev_pos_span = elev_max - (elev_center + elev_dz);
	elev_neg_span = (elev_center - elev_dz) - elev_min;
    }

    double roll_cmd = 0.0;
    if ( aileron >= ail_center + ail_dz ) {
	roll_cmd = (aileron - (ail_center + ail_dz)) / ail_pos_span;
	if ( roll_cmd > 1.0 ) { roll_cmd = 1.0; }
    } else if ( aileron <= ail_center - ail_dz ) {
	roll_cmd = (aileron - (ail_center - ail_dz)) / ail_neg_span;
	if ( roll_cmd < -1.0 ) { roll_cmd = -1.0; }
    }
    int roll_sign = 1;
    if ( roll_cmd < 0 ) { roll_sign = -1; roll_cmd *= -1.0; }

    double roll_delta = 0.0;
    double new_roll = targets_node.getDouble("roll_deg");
    if ( fabs(roll_cmd) > 0.001 ) {
	// pilot is inputing a roll command
	roll_delta = roll_sign * roll_cmd
	    * cas_aileron_node.getDouble("full_rate_degps") * dt;
    } else {
	// pilot stick is centered
	if ( fabs(new_roll) <= 10.0 ) {
	    // target roll within +/- 10 degrees of level, roll slowly
	    // back to level at 2 degps rate.
	    roll_delta = -new_roll;
	    if ( roll_delta > 2.0 * dt ) {
		roll_delta = 2.0 * dt;
	    } else if ( roll_delta < -2.0 * dt ) {
		roll_delta = -2.0 * dt;
	    }
	}
    }

    new_roll += roll_delta;
    if ( new_roll < -45.0 ) { new_roll = -45.0; }
    if ( new_roll > 45.0 ) { new_roll = 45.0; }
    targets_node.setDouble( "roll_deg", new_roll );

    double pitch_cmd = 0.0;
    if ( elevator >= elev_center + elev_dz ) {
	pitch_cmd = (elevator - (elev_center + elev_dz)) / elev_pos_span;
	if ( pitch_cmd > 1.0 ) { pitch_cmd = 1.0; }
    } else if ( elevator <= elev_center - elev_dz ) {
	pitch_cmd = (elevator - (elev_center - elev_dz)) / elev_neg_span;
	if ( pitch_cmd < -1.0 ) { pitch_cmd = -1.0; }
    }
    int pitch_sign = 1;
    if ( pitch_cmd < 0 ) { pitch_sign = -1; pitch_cmd *= -1.0; }

    double pitch_delta = pitch_sign * pitch_cmd /* * pitch_cmd */
	* cas_elevator_node.getDouble("full_rate_degps") * dt;

    double new_pitch_base
	= targets_node.getDouble("pitch_base_deg") + pitch_delta;
    if ( new_pitch_base < -15.0 ) { new_pitch_base = -15.0; }
    if ( new_pitch_base > 15.0 ) { new_pitch_base = 15.0; }
    targets_node.setDouble( "pitch_base_deg", new_pitch_base );

    // map throttle [0 ... 1] to [-mp ... mp] for throttle pitch offset
    // where mp is the max pitch bias.  This "simulates" the natural
    // tendency of an aircraft to pitch up or down as throttle is
    // manipulated.
    const double mp = 4.0; 	// degrees throttle pitch bias
    double pitch_throttle_delta
	= (pilot_node.getDouble("throttle") * 2.0 - 1.0) * mp;
    double new_pitch = new_pitch_base + pitch_throttle_delta;
    targets_node.setDouble( "pitch_deg", new_pitch );

    // this is a hard coded hack, but pass through throttle and rudder here
    engine_node.setDouble( "throttle", pilot_node.getDouble("throttle") );
    flight_node.setDouble( "rudder", pilot_node.getDouble("rudder") );

#if 0
    static int ii = 0;
    if (ii > 10) {
	printf("ail=%.3f elev=%.3f cmd: roll=%.3f pitch=%.3f\n",
	       aileron, elevator, roll_cmd, pitch_cmd);
	printf("targets: roll=%.3f pitch=%.3f\n", new_roll, new_pitch);
	ii = 0;
    } else {
	ii++;
    }
#endif
}
