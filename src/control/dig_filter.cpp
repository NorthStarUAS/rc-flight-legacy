// dig_filter.cpp - a flexible digital filter class
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

#include "dig_filter.h"


AuraDigitalFilter::AuraDigitalFilter( string config_path )
{
    size_t pos;
    samples = 1;

    component_node = pyGetNode(config_path, true);
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

    if ( component_node.hasChild("type") ) {
	string cval = component_node.getString("type");
	if ( cval == "exponential" ) {
	    filterType = exponential;
	} else if (cval == "double-exponential") {
	    filterType = doubleExponential;
	} else if (cval == "moving-average") {
	    filterType = movingAverage;
	} else if (cval == "noise-spike") {
	    filterType = noiseSpike;
	}
    }
    if ( component_node.hasChild("filter_time") ) {
	Tf = component_node.getDouble("filter_time");
    }
    if ( component_node.hasChild("samples") ) {
	samples = component_node.getLong("samples");
    }
    if ( component_node.hasChild("max_rate_of_change") ) {
	rateOfChange = component_node.getDouble("max_rate_of_change");
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

    output.resize(2, 0.0);
    input.resize(samples + 1, 0.0);
}

void AuraDigitalFilter::reset() {
}

void AuraDigitalFilter::update(double dt)
{
    // test if all of the provided enable flags are true
    enabled = true;
    for ( unsigned int i = 0; i < enables_node.size(); i++ ) {
        if ( !enables_node[i].getBool(enables_attr[i].c_str()) ) {
            enabled = false;
            break;
        }
    }

    input.push_front( input_node.getDouble(input_attr.c_str()) );
    input.resize(samples + 1, 0.0);

    if ( enabled && dt > 0.0 ) {
        /*
         * Exponential filter
         *
         * Output[n] = alpha*Input[n] + (1-alpha)*Output[n-1]
         *
         */

        if (filterType == exponential)
        {
            double alpha = 1 / ((Tf/dt) + 1);
            output.push_front(alpha * input[0] + 
                              (1 - alpha) * output[0]);
	    for ( unsigned int i = 0; i < output_node.size(); i++ ) {
		output_node[i].setDouble( output_attr[i].c_str(), output[0] );
	    }
            output.resize(1);
        } 
        else if (filterType == doubleExponential)
        {
            double alpha = 1 / ((Tf/dt) + 1);
            output.push_front(alpha * alpha * input[0] + 
                              2 * (1 - alpha) * output[0] -
                              (1 - alpha) * (1 - alpha) * output[1]);
 	    for ( unsigned int i = 0; i < output_node.size(); i++ ) {
		output_node[i].setDouble( output_attr[i].c_str(), output[0] );
	    }
            output.resize(2);
        }
        else if (filterType == movingAverage)
        {
            output.push_front(output[0] + 
                              (input[0] - input.back()) / samples);
 	    for ( unsigned int i = 0; i < output_node.size(); i++ ) {
		output_node[i].setDouble( output_attr[i].c_str(), output[0] );
	    }
            output.resize(1);
        }
        else if (filterType == noiseSpike)
        {
            double maxChange = rateOfChange * dt;

            if ((output[0] - input[0]) > maxChange)
            {
                output.push_front(output[0] - maxChange);
            }
            else if ((output[0] - input[0]) < -maxChange)
            {
                output.push_front(output[0] + maxChange);
            }
            else if (fabs(input[0] - output[0]) <= maxChange)
            {
                output.push_front(input[0]);
            }

 	    for ( unsigned int i = 0; i < output_node.size(); i++ ) {
		output_node[i].setDouble( output_attr[i].c_str(), output[0] );
	    }
	    output.resize(1);
        }
        if ( component_node.getBool("debug") ) {
            printf("input: %.3f\toutput: %.3f\n", input[0], output[0]);
        }
    }
}
