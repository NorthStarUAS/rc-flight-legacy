// pid.cpp - a standard pid controller
//
// Written by Curtis Olson, started January 2004.
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


#include <math.h>

#include <pyprops.h>

#include "pid.h"


AuraPID::AuraPID( string config_path ):
    do_reset(true),
    proportional( false ),
    integral( false ),
    iterm( 0.0 ),
    y_n( 0.0 ),
    y_n_1( 0.0 ),
    r_n( 0.0 )
{
    size_t pos;

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

    // reference
    node = component_node.getChild("reference", true);
    string ref_prop = node.getString("prop");
    ref_value = node.getString("value");
    pos = ref_prop.rfind("/");
    if ( pos != string::npos ) {
	string path = ref_prop.substr(0, pos);
	ref_attr = ref_prop.substr(pos+1);
	// printf("path = %s attr = %s\n", path.c_str(), ref_attr.c_str());
	ref_node = pyGetNode( path, true );
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


void AuraPID::reset() {
    do_reset = true;
}


void AuraPID::update( double dt ) {
    // test if all of the provided enable flags are true
    enabled = true;
    for ( unsigned int i = 0; i < enables_node.size(); i++ ) {
        if ( !enables_node[i].getBool(enables_attr[i].c_str()) ) {
            enabled = false;
            break;
        }
    }

    bool debug = component_node.getBool("debug");
    if ( debug ) printf("Updating %s\n", get_name().c_str());
    y_n = input_node.getDouble(input_attr.c_str());

    double r_n = 0.0;
    if ( ref_value != "" ) {
	// printf("nonzero ref_value\n");
	r_n = atof(ref_value.c_str());
    } else {
	r_n = ref_node.getDouble(ref_attr.c_str());
    }
                      
    double error = r_n - y_n;

    string wrap = component_node.getString("wrap");
    if ( wrap == "180" ) {
        // wrap error (by +/- 360 degrees to put the result in [-180, 180]
        if ( error < -180 ) { error += 360; }
        if ( error > 180 ) { error -= 360; }
    } else if ( wrap == "pi" ) {
        // wrap error (by +/- 2*pi degrees to put the result in [-pi, pi]
        if ( error < -M_PI ) { error += 2*M_PI; }
        if ( error > M_PI ) { error -= 2*M_PI; }
    }
    
    if ( debug ) printf("input = %.3f reference = %.3f error = %.3f\n",
			y_n, r_n, error);

    double u_trim = config_node.getDouble("u_trim");
    double u_min = config_node.getDouble("u_min");
    double u_max = config_node.getDouble("u_max");

    double Kp = config_node.getDouble("Kp");
    double Ti = config_node.getDouble("Ti");
    double Td = config_node.getDouble("Td");
    double Ki = 0.0;
    if ( Ti > 0.0001 ) {
	Ki = Kp / Ti;
    }
    double Kd = Kp * Td;

    // proportional term
    double pterm = Kp * error + u_trim;

    // integral term
    if ( Ti > 0.0001 ) {
        iterm += Ki * error * dt;
    } else {
        iterm = 0.0;
    }
    
    // if the reset flag is set, back compute an iterm that will
    // produce zero initial transient (overwriting the existing
    // iterm) then unset the do_reset flag.
    if ( do_reset ) {
        if ( Ti > 0.0001 ) {
            double u_n = output_node[0].getDouble(output_attr[0].c_str());
            // and clip
            double u_min = config_node.getDouble("u_min");
            double u_max = config_node.getDouble("u_max");
            if ( u_n < u_min ) { u_n = u_min; }
            if ( u_n > u_max ) { u_n = u_max; }
            iterm = u_n - pterm;
        } else {
            iterm = 0.0;
        }
        do_reset = false;
    }
    
    // derivative term: observe that dError/dt = -dInput/dt (except
    // when the setpoint changes (which we don't want to react to
    // anyway.)  This approach avoids "derivative kick" when the set
    // point changes.
    double dy = y_n - y_n_1;
    y_n_1 = y_n;
    double dterm = Kd * -dy / dt;

    double output = pterm + iterm + dterm;
    if ( output < u_min ) {
        if ( Ti > 0.0001 ) {
            iterm += u_min - output;
        }
	output = u_min;
    }
    if ( output > u_max ) {
        if ( Ti > 0.0001 ) {
            iterm -= output - u_max;
        }
	output = u_max;
    }

    if ( debug ) printf("pterm = %.3f iterm = %.3f\n",
			pterm, iterm);

    if ( !enabled ) {
        // this will force a reset when component becomes enabled
        do_reset = true;
    } else {
	// Copy the result to the output node(s)
	for ( unsigned int i = 0; i < output_node.size(); i++ ) {
	    output_node[i].setDouble( output_attr[i].c_str(), output );
	}
    }
}


