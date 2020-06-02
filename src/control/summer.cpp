// summer.cpp - a component to sum the output of previous components
//
// Copyright (C) 2017  Curtis L. Olson  - curtolson@flightgear.org
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

#include "summer.h"


AuraSummer::AuraSummer ( string config_path )
{
    size_t pos;

    component_node = pyGetNode(config_path);
    vector <string> children;
    
    // enable
    pyPropertyNode node = component_node.getChild( "enable", true );
    children = node.getChildren();
    printf("enables: %d prop(s)\n", children.size());
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
    node = component_node.getChild( "input", true );
    children = node.getChildren();
    for ( unsigned int i = 0; i < children.size(); ++i ) {
	if ( children[i].substr(0,4) == "prop" ) {
	    string input_prop = node.getString(children[i].c_str());
	    pos = input_prop.rfind("/");
	    if ( pos != string::npos ) {
		string path = input_prop.substr(0, pos);
		string attr = input_prop.substr(pos+1);
		pyPropertyNode onode = pyGetNode( path, true );
		input_node.push_back( onode );
		input_attr.push_back( attr );
	    } else {
		printf("WARNING: requested bad input path: %s\n",
		       input_prop.c_str());
	    }
	} else {
	    printf("WARNING: unknown tag in input section: %s\n",
		   children[i].c_str());
	}
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
    
    // config
    config_node = component_node.getChild( "config", true );
}

void AuraSummer::reset() {
    // noop
}

void AuraSummer::update( double dt ) {
    // test if all of the provided enable flags are true
    enabled = true;
    for ( unsigned int i = 0; i < enables_node.size(); i++ ) {
        if ( !enables_node[i].getBool(enables_attr[i].c_str()) ) {
            enabled = false;
            break;
        }
    }

    if ( enabled ) {
	bool debug = component_node.getBool("debug");
	if ( debug ) printf("Updating %s\n", get_name().c_str());
	double sum = 0.0;
	for ( unsigned int i = 0; i < input_node.size(); i++ ) {
	    double val = input_node[i].getDouble( input_attr[i].c_str() );
	    sum += val;
	    if (debug) printf("  %s = %.3f\n", input_attr[i].c_str(), val);
	}
	if ( config_node.hasChild("u_min") ) {
	    double u_min = config_node.getDouble("u_min");
	    if ( sum < u_min ) { sum = u_min; }
	}
	if ( config_node.hasChild("u_max") ) {
	    double u_max = config_node.getDouble("u_max");
	    if ( sum > u_max ) { sum = u_max; }
	}
	if (debug) printf("  sum = %.3f\n", sum);
	for ( unsigned int i = 0; i < output_node.size(); i++ ) {
	    output_node[i].setDouble( output_attr[i].c_str(), sum );
	}
    }
}
