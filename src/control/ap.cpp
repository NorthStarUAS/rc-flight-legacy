// ap.cpp - a flexible generic way to build autopilots
//
// Written by Curtis Olson, started January 2004.
//
// Copyright (C) 2004-2017  Curtis L. Olson - curtolson@flightgear.org
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


#include <props2.h>

#include <math.h>
#include <stdlib.h>

#include <string>
#include <sstream>
using std::string;
using std::ostringstream;

#include "ap.h"
#include "dig_filter.h"
#include "dtss.h"
#include "pid.h"
#include "pid_vel.h"
#include "predictor.h"
#include "summer.h"


void AuraAutopilot::init() {
    if ( ! build() ) {
	printf("AP: Detected an internal inconsistency in the autopilot\n");
	printf("configuration.  See earlier errors for details.\n" );
	exit(-1);
    }
}


void AuraAutopilot::reset() {
    for ( unsigned int i = 0; i < components.size(); ++i ) {
        components[i]->reset();
    }
}


bool AuraAutopilot::build() {
    PropertyNode config_node("/config/autopilot");
    for ( int i = 0; i < config_node.getLen("component"); i++ ) {
        printf("Number of components: %d (i: %d)\n", config_node.getLen("component"), i);
        string child_name = "component/" + std::to_string(i);
	PropertyNode component = config_node.getChild(child_name.c_str());
        printf("ap stage: %s\n", child_name.c_str());
        string config_path = "/config/autopilot/" + child_name;
        string module = component.getString("module");
        if ( module == "pid" ) {
            APComponent *c
                = new AuraPID( config_path );
            components.push_back( c );
        } else if ( module == "pid_velocity" ) {
            APComponent *c
                = new AuraPIDVel( config_path );
            components.push_back( c );
        } else if ( module == "dtss" ) {
            APComponent *c
                = new AuraDTSS( config_path );
            components.push_back( c );
        } else if ( module == "predict_simple" ) {
            APComponent *c
                = new AuraPredictor( config_path );
            components.push_back( c );
        } else if ( module == "filter" ) {
            APComponent *c
                = new AuraDigitalFilter( config_path );
            components.push_back( c );
        } else if ( module == "summer" ) {
            APComponent *c
                = new AuraSummer( config_path );
            components.push_back( c );
        } else {
            printf("Unknown AP module name: %s\n", module.c_str());
            return false;
        }
    }

    return true;
}


// normalize a value to lie between min and max
template <class T>
inline void SG_NORMALIZE_RANGE( T &val, const T min, const T max ) {
    T step = max - min;
    while( val >= max )  val -= step;
    while( val < min ) val += step;
};


/*
 * Update the list of autopilot components
 */

void AuraAutopilot::update( double dt ) {
    for ( unsigned int i = 0; i < components.size(); ++i ) {
        components[i]->update( dt );
    }
}
