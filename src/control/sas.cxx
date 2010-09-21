// sas.cxx - stability assist system (aka fly-by-wire)
//
// Written by Curtis Olson, started September 2010.
//
// Copyright (C) 2010  Curtis L. Olson  - http://www.atiak.com
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//
// $Id: route_mgr.hxx,v 1.11 2008/11/23 04:02:17 curt Exp $


#include <math.h>		// fabs()
#include <stdio.h>

#include "comms/logging.h"
#include "util/timing.h"

#include "sas.hxx"


UGSAS sas;			// global sas object


UGSAS::UGSAS() :
    sas_mode( PassThrough )
{
}

UGSAS::~UGSAS() {
}


// bind property nodes
void UGSAS::bind() {
    pilot_aileron_node = fgGetNode("/sensors/pilot/aileron", true);
    pilot_elevator_node = fgGetNode("/sensors/pilot/elevator", true);
    pilot_throttle_node = fgGetNode("/sensors/pilot/throttle", true);
    pilot_rudder_node = fgGetNode("/sensors/pilot/rudder", true);

    aileron_min_node = fgGetNode("/config/sas/aileron/min", true);
    aileron_max_node = fgGetNode("/config/sas/aileron/max", true);
    aileron_center_node = fgGetNode("/config/sas/aileron/center", true);
    aileron_dz_node = fgGetNode("/config/sas/aileron/dead-zone", true);
    aileron_full_rate_node
	= fgGetNode("/config/sas/aileron/full-rate-degps", true);

    elevator_min_node = fgGetNode("/config/sas/elevator/min", true);
    elevator_max_node = fgGetNode("/config/sas/elevator/max", true);
    elevator_center_node = fgGetNode("/config/sas/elevator/center", true);
    elevator_dz_node = fgGetNode("/config/sas/aileron/dead-zone", true);
    elevator_full_rate_node
	= fgGetNode("/config/sas/elevator/full-rate-degps", true);

    target_roll_deg_node
	= fgGetNode("/autopilot/settings/target-roll-deg", true);
    target_pitch_deg_node
	= fgGetNode("/autopilot/settings/target-pitch-deg", true);
    target_pitch_base_deg_node
	= fgGetNode("/autopilot/settings/target-pitch-base-deg", true);

    throttle_output_node = fgGetNode("/controls/engine/throttle", true);
    rudder_output_node = fgGetNode("/controls/flight/rudder", true);

    ap_master_switch_node = fgGetNode("/autopilot/master-switch", true);
    sas_mode_node = fgGetNode("/autopilot/sas-mode", true);
}

// initialize SAS system
void UGSAS::init() {
    bind();
}

void UGSAS::update() {
    double current_time = get_Time();
    double dt = current_time - last_time;
    last_time = current_time;

    if ( !ap_master_switch_node->getBoolValue() ) {
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

    double aileron = pilot_aileron_node->getDoubleValue();
    double elevator = pilot_elevator_node->getDoubleValue();

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
	ail_max = aileron_max_node->getDoubleValue();
	ail_min = aileron_min_node->getDoubleValue();
	ail_center = aileron_center_node->getDoubleValue();
	ail_dz = aileron_dz_node->getDoubleValue();
	ail_pos_span = ail_max - (ail_center + ail_dz);
	ail_neg_span = (ail_center - ail_dz) - ail_min;

	elev_max = elevator_max_node->getDoubleValue();
	elev_min = elevator_min_node->getDoubleValue();
	elev_center = elevator_center_node->getDoubleValue();
	elev_dz = elevator_dz_node->getDoubleValue();
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
    double new_roll = target_roll_deg_node->getDoubleValue();
    if ( fabs(roll_cmd) > 0.001 ) {
	// pilot is inputing a roll command
	roll_delta
	    = roll_sign * roll_cmd * aileron_full_rate_node->getDoubleValue()
	      * dt;
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
    target_roll_deg_node->setDoubleValue( new_roll );

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

    double pitch_delta
	= pitch_sign * pitch_cmd /* * pitch_cmd */
	* elevator_full_rate_node->getDoubleValue() * dt;

    double new_pitch_base
	= target_pitch_base_deg_node->getDoubleValue() + pitch_delta;
    if ( new_pitch_base < -15.0 ) { new_pitch_base = -15.0; }
    if ( new_pitch_base > 15.0 ) { new_pitch_base = 15.0; }
    target_pitch_base_deg_node->setDoubleValue( new_pitch_base );

    // map throttle [0 ... 1] to [-mp ... mp] for throttle pitch offset
    // where mp is the max pitch bias.  This "simulates" the natural
    // tendency of an aircraft to pitch up or down as throttle is
    // manipulated.
    const double mp = 2.5; 	// degrees throttle pitch bias
    double pitch_throttle_delta
	= (pilot_throttle_node->getFloatValue() * 2.0 - 1.0) * mp;
    double new_pitch = new_pitch_base + pitch_throttle_delta;
    target_pitch_deg_node->setDoubleValue( new_pitch );

    // this is a hard coded hack, but pass through throttle and rudder here
    throttle_output_node->setFloatValue( pilot_throttle_node->getFloatValue() );
    rudder_output_node->setFloatValue( pilot_rudder_node->getFloatValue() );

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
