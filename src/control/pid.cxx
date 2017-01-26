// pid.cxx - a standard pid controller
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


#include "python/pyprops.hxx"

#include "pid.hxx"


AuraPID::AuraPID( string config_path ):
    proportional( false ),
    integral( false ),
    iterm( 0.0 ),
    clamp( false ),
    y_n( 0.0 ),
    y_n_1( 0.0 ),
    r_n( 0.0 )
{
    size_t pos;

    component_node = pyGetNode(config_path, true);

    // enable
    pyPropertyNode node = component_node.getChild("enable", true);
    string enable_prop = node.getString("prop");
    enable_value = node.getString("value");
    honor_passive = node.getBool("honor_passive");
    pos = enable_prop.rfind("/");
    if ( pos != string::npos ) {
	string path = enable_prop.substr(0, pos);
	enable_attr = enable_prop.substr(pos+1);
	enable_node = pyGetNode( path, true );
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
	printf("path = %s attr = %s\n", path.c_str(), ref_attr.c_str());
	ref_node = pyGetNode( path, true );
    }

    // output
    node = component_node.getChild( "output", true );
    vector <string> children = node.getChildren();
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


void AuraPID::update( double dt ) {
    if (!enable_node.isNull() && enable_node.getString(enable_attr.c_str()) == enable_value) {
	enabled = true;
    } else {
	enabled = false;
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
    iterm += Ki * error * dt;
    
    // derivative term: observe that dError/dt = -dInput/dt (except
    // when the setpoint changes (which we don't want to react to
    // anyway.)  This approach avoids "derivative kick" when the set
    // point changes.
    double dy = y_n - y_n_1;
    y_n_1 = y_n;
    double dterm = Kd * -dy / dt;

    double output = pterm + iterm + dterm;
    if ( output < u_min ) {
	iterm += u_min - output;
	output = u_min;
    }
    if ( output > u_max ) {
	iterm -= output - u_max;
	output = u_max;
    }

    if ( debug ) printf("pterm = %.3f iterm = %.3f\n",
			pterm, iterm);
    if ( debug ) printf("clamped output = %.3f\n", output);

    if ( enabled ) {
	// Copy the result to the output node(s)
	for ( unsigned int i = 0; i < output_node.size(); i++ ) {
	    output_node[i].setDouble( output_attr[i].c_str(), output );
	}
    } else {
	// Force iterm to zero so we don't activate with maximum windup
	iterm = 0.0;
    }
}


