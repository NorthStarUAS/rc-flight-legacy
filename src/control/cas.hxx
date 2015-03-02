// cas.hxx - Command Augmentation System (aka fly-by-wire)
//
// Written by Curtis Olson, started September 2010.
//
// Copyright (C) 2010  Curtis L. Olson  - http://www.atiak.com
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


#ifndef _CAS_HXX
#define _CAS_HXX

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

class UGCAS {

public:

    enum ugCASMode {
        PassThrough = 0,
        PitchRollRates = 1
    };

private:

    // route configuration
    SGPropertyNode *config_props;

    // property nodes

    SGPropertyNode *pilot_aileron_node;
    SGPropertyNode *pilot_elevator_node;
    SGPropertyNode *pilot_throttle_node;
    SGPropertyNode *pilot_rudder_node;

    SGPropertyNode *aileron_min_node;
    SGPropertyNode *aileron_max_node;
    SGPropertyNode *aileron_center_node;
    SGPropertyNode *aileron_dz_node;
    SGPropertyNode *aileron_full_rate_node;

    SGPropertyNode *elevator_min_node;
    SGPropertyNode *elevator_max_node;
    SGPropertyNode *elevator_center_node;
    SGPropertyNode *elevator_dz_node;
    SGPropertyNode *elevator_full_rate_node;

    SGPropertyNode *target_roll_deg_node;
    SGPropertyNode *target_pitch_deg_node;
    SGPropertyNode *target_pitch_base_deg_node;

    SGPropertyNode *throttle_output_node;
    SGPropertyNode *rudder_output_node;

    // fcs mode
    SGPropertyNode *ap_master_switch_node;
    SGPropertyNode *cas_mode_node;

    ugCASMode cas_mode;

    double last_time;

public:

    UGCAS();
    ~UGCAS();

    void bind();

    void init();

    void update();

    inline void set_cas_mode( ugCASMode mode ) {
	cas_mode = mode;
    }
 
    inline ugCASMode get_cas_mode() {
        return cas_mode;
    }
};


extern UGCAS cas;		// global CAS object


#endif // _CAS_HXX
