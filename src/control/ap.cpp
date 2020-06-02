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


#include <pyprops.h>

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
    pyPropertyNode config_props = pyGetNode( "/config/autopilot", true );

    // FIXME: we have always depended on the order of children
    // components here to ensure PID stages are run in the correct
    // order, however that is a bad thing to assume ... especially now
    // with pyprops!!!
    vector <string> children = config_props.getChildren();
    for ( unsigned int i = 0; i < children.size(); ++i ) {
	pyPropertyNode component = config_props.getChild(children[i].c_str(),
							 true);
        printf("ap stage: %s\n", children[i].c_str());
	string name = children[i];
	size_t pos = name.find("[");
	if ( pos != string::npos ) {
	    name = name.substr(0, pos);
	}
	if ( name == "component" ) {
	    ostringstream config_path;
	    config_path << "/config/autopilot/" << children[i];
	    string module = component.getString("module");
            if ( module == "pid" ) {
		APComponent *c
		    = new AuraPID( config_path.str() );
		components.push_back( c );
	    } else if ( module == "pid_velocity" ) {
		APComponent *c
		    = new AuraPIDVel( config_path.str() );
		components.push_back( c );
	    } else if ( module == "dtss" ) {
		APComponent *c
		    = new AuraDTSS( config_path.str() );
		components.push_back( c );
	    } else if ( module == "predict_simple" ) {
		APComponent *c
		    = new AuraPredictor( config_path.str() );
		components.push_back( c );
	    } else if ( module == "filter" ) {
		APComponent *c
		    = new AuraDigitalFilter( config_path.str() );
		components.push_back( c );
	    } else if ( module == "summer" ) {
		APComponent *c
		    = new AuraSummer( config_path.str() );
		components.push_back( c );
	    } else {
		printf("Unknown AP module name: %s\n", module.c_str());
		return false;
	    }
	} else if ( name == "L1_controller" ) {
	    // configuration placeholder, we don't do anything here.
	} else if ( name == "TECS" ) {
            // configuration placeholder, we don't do anything here.
         } else {
	    printf("Unknown top level section: %s\n", children[i].c_str() );
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

