// sas.hxx - stability assist system (aka fly-by-wire)
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


#ifndef _SAS_HXX
#define _SAS_HXX

#ifndef __cplusplus
# error This library requires C++
#endif

#include <string>
#include <vector>

using std::string;
using std::vector;

#include "props/props.hxx"
#include "route.hxx"


/**
 * Top level route manager class
 * 
 */

class UGSAS {

public:

    enum ugSASMode {
        PassThrough = 0,
        PitchRollRates = 1
    };

private:

    // route configuration
    SGPropertyNode_ptr config_props;

    // property nodes

    SGPropertyNode_ptr pilot_aileron_node;
    SGPropertyNode_ptr pilot_elevator_node;
    SGPropertyNode_ptr pilot_throttle_node;
    SGPropertyNode_ptr pilot_rudder_node;

    SGPropertyNode_ptr aileron_min_node;
    SGPropertyNode_ptr aileron_max_node;
    SGPropertyNode_ptr aileron_center_node;
    SGPropertyNode_ptr aileron_dz_node;
    SGPropertyNode_ptr aileron_full_rate_node;

    SGPropertyNode_ptr elevator_min_node;
    SGPropertyNode_ptr elevator_max_node;
    SGPropertyNode_ptr elevator_center_node;
    SGPropertyNode_ptr elevator_dz_node;
    SGPropertyNode_ptr elevator_full_rate_node;

    SGPropertyNode_ptr target_roll_deg_node;
    SGPropertyNode_ptr target_pitch_deg_node;
    SGPropertyNode_ptr target_pitch_base_deg_node;

    SGPropertyNode_ptr throttle_output_node;
    SGPropertyNode_ptr rudder_output_node;

    // fcs mode
    SGPropertyNode_ptr ap_master_switch_node;
    SGPropertyNode_ptr sas_mode_node;

    ugSASMode sas_mode;

    double last_time;

public:

    UGSAS();
    ~UGSAS();

    void bind();

    void init();

    void update();

    inline void set_sas_mode( ugSASMode mode ) {
	sas_mode = mode;
    }
 
    inline ugSASMode get_sas_mode() {
        return sas_mode;
    }
};


extern UGSAS sas;		// global SAS object


#endif // _SAS_HXX
