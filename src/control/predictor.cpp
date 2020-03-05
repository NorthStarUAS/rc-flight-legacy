// predictor.cpp - predict a future sensor value
//
// Copyright (C) 2004-2017  Curtis L. Olson  - curtolson@flightgear.org
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

#include "predictor.h"


AuraPredictor::AuraPredictor ( string config_path ):
    last_value ( 999999999.9 ),
    average ( 0.0 ),
    seconds( 0.0 ),
    filter_gain( 0.0 ),
    ivalue( 0.0 )
{
    size_t pos;

    component_node = pyGetNode(config_path);
    vector <string> children;
    
    // enable
    pyPropertyNode node = component_node.getChild( "enable", true );
    children = node.getChildren();
    printf("enables: %ld prop(s)\n", children.size());
    for ( unsigned int i = 0; i < children.size(); ++i ) {
	if ( children[i].substr(0,4) == "prop" ) {
	    string enable_prop = node.getString(children[i].c_str());
            printf("  %s\n", enable_prop.c_str());
	    pos = enable_prop.rfind("/");
	    if ( pos != string::npos ) {
		string path = enable_prop.substr(0, pos);
		string attr = enable_prop.substr(pos+1);
		pyPropertyNode en_node = pyGetNode( path, true );
		enables_node.push_back( en_node );
		enables_attr.push_back( attr );
	    } else {
		printf("WARNING: requested bad enable path: %s\n",
		       enable_prop.c_str());
	    }
	} else {
	    printf("WARNING: unknown tag in enable section: %s\n",
		   children[i].c_str());
	}
    }

    // input
    node = component_node.getChild("input", true);
    string input_prop = node.getString("prop");
    pos = input_prop.rfind("/");
    if ( pos != string::npos ) {
	string path = input_prop.substr(0, pos);
	input_attr = input_prop.substr(pos+1);
	input_node = pyGetNode( path, true );
    }

    if ( component_node.hasChild("seconds") ) {
	seconds = component_node.getDouble("seconds");
    }
    if ( component_node.hasChild("filter_gain") ) {
	filter_gain = component_node.getDouble("filter_gain");
    }
    
    // output
    node = component_node.getChild( "output", true );
    children = node.getChildren();
    for ( unsigned int i = 0; i < children.size(); ++i ) {
	if ( children[i].substr(0,4) == "prop" ) {
	    string output_prop = node.getString(children[i].c_str());
	    pos = output_prop.rfind("/");
	    if ( pos != string::npos ) {
		string path = output_prop.substr(0, pos);
		string attr = output_prop.substr(pos+1);
		pyPropertyNode onode = pyGetNode( path, true );
		output_node.push_back( onode );
		output_attr.push_back( attr );
	    } else {
		printf("WARNING: requested bad output path: %s\n",
		       output_prop.c_str());
	    }
	} else {
	    printf("WARNING: unknown tag in output section: %s\n",
		   children[i].c_str());
	}
    }
}

void AuraPredictor::reset() {
}


void AuraPredictor::update( double dt ) {
    /*
       Simple moving average filter converts input value to predicted value "seconds".

       Smoothing as described by Curt Olson:
         gain would be valid in the range of 0 - 1.0
         1.0 would mean no filtering.
         0.0 would mean no input.
         0.5 would mean (1 part past value + 1 part current value) / 2
         0.1 would mean (9 parts past value + 1 part current value) / 10
         0.25 would mean (3 parts past value + 1 part current value) / 4

    */

    // test if all of the provided enable flags are true
    enabled = true;
    for ( unsigned int i = 0; i < enables_node.size(); i++ ) {
        if ( !enables_node[i].getBool(enables_attr[i].c_str()) ) {
            enabled = false;
            break;
        }
    }

    ivalue = input_node.getDouble(input_attr.c_str());

    if ( enabled ) {
        // first time initialize average
        if (last_value >= 999999999.0) {
           last_value = ivalue;
        }

        if ( dt > 0.0 ) {
            double current = (ivalue - last_value)/dt; // calculate current error change (per second)
            if ( dt < 1.0 ) {
                average = (1.0 - dt) * average + current * dt;
            } else {
                average = current;
            }

            // calculate output with filter gain adjustment
            double output = ivalue + (1.0 - filter_gain) * (average * seconds) + filter_gain * (current * seconds);

	    // Copy the result to the output node(s)
	    for ( unsigned int i = 0; i < output_node.size(); i++ ) {
		output_node[i].setDouble( output_attr[i].c_str(), output );
	    }
        }
        last_value = ivalue;
    }
}
