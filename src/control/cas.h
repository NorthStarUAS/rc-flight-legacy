// cas.h - Command Augmentation System (aka fly-by-wire)
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


#pragma once

#include <pyprops.h>

#include <string>
#include <vector>

using std::string;
using std::vector;

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

    // property nodes
    pyPropertyNode config_node;
    pyPropertyNode pilot_node;
    pyPropertyNode cas_aileron_node;
    pyPropertyNode cas_elevator_node;
    pyPropertyNode targets_node;
    pyPropertyNode flight_node;
    pyPropertyNode engine_node;
    
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
