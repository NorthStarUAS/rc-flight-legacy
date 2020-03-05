// pid_vel.cpp - a pid controller in the velocity form
//
// Written by Curtis Olson and Roy Vegard Ovesen, started January 2004.
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

#include "pid_vel.h"


AuraPIDVel::AuraPIDVel( string config_path ):
    ep_n_1( 0.0 ),
    edf_n_1( 0.0 ),
    edf_n_2( 0.0 ),
    u_n_1( 0.0 ),
    desiredTs( 0.00001 ),
    elapsedTime( 0.0 )
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
	printf("path = %s attr = %s\n", path.c_str(), input_attr.c_str());
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
    children = node.getChildren();
    for ( unsigned int i = 0; i < children.size(); ++i ) {
	if ( children[i].substr(0,4) == "prop" ) {
	    string output_prop = node.getString(children[i].c_str());
	    pos = output_prop.rfind("/");
	    if ( pos != string::npos ) {
		string path = output_prop.substr(0, pos);
		string attr = output_prop.substr(pos+1);
		printf("path = %s attr = %s\n", path.c_str(), attr.c_str());
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
    if ( config_node.hasChild("Ts") ) {
	desiredTs = config_node.getDouble("Ts");
    }
            
    if ( !config_node.hasChild("beta") ) {
	// create with default value
	config_node.setDouble( "beta", 1.0 );
    }
    if ( !config_node.hasChild("alpha") ) {
	// create with default value
	config_node.setDouble( "alpha", 0.1 );
    }
}


void AuraPIDVel::reset() {
}


/*
 * Roy Vegard Ovesen:
 *
 * Ok! Here is the PID controller algorithm that I would like to see
 * implemented:
 *
 *   delta_u_n = Kp * [ (ep_n - ep_n-1) + ((Ts/Ti)*e_n)
 *               + (Td/Ts)*(edf_n - 2*edf_n-1 + edf_n-2) ]
 *
 *   u_n = u_n-1 + delta_u_n
 *
 * where:
 *
 * delta_u : The incremental output
 * Kp      : Proportional gain
 * ep      : Proportional error with reference weighing
 *           ep = beta * (r - y)
 *           where:
 *           beta : Weighing factor
 *           r    : Reference (setpoint)
 *           y    : Process value, measured
 * e       : Error
 *           e = r - y
 * Ts      : Sampling interval
 * Ti      : Integrator time
 * Td      : Derivator time
 * edf     : Derivate error with reference weighing and filtering
 *           edf_n = edf_n-1 / ((Ts/Tf) + 1) + ed_n * (Ts/Tf) / ((Ts/Tf) + 1)
 *           where:
 *           Tf : Filter time
 *           Tf = alpha * Td , where alpha usually is set to 0.1
 *           ed : Unfiltered derivate error with reference weighing
 *             ed = gamma * r - y
 *             where:
 *             gamma : Weighing factor
 * 
 * u       : absolute output
 * 
 * Index n means the n'th value.
 * 
 * 
 * Inputs:
 * enabled ,
 * y_n , r_n , beta=1 , gamma=0 , alpha=0.1 ,
 * Kp , Ti , Td , Ts (is the sampling time available?)
 * u_min , u_max
 * 
 * Output:
 * u_n
 */

void AuraPIDVel::update( double dt ) {
    double ep_n;            // proportional error with reference weighing
    double e_n;             // error
    double ed_n;            // derivative error
    double edf_n;           // derivative error filter
    double Tf;              // filter time
    double delta_u_n = 0.0; // incremental output
    double u_n = 0.0;       // absolute output
    double Ts;              // sampling interval (sec)
    
    elapsedTime += dt;
    if ( elapsedTime <= desiredTs ) {
        // do nothing if no time has elapsed
        return;
    }
    Ts = elapsedTime;
    elapsedTime = 0.0;

    // test if all of the provided enable flags are true
    enabled = true;
    for ( unsigned int i = 0; i < enables_node.size(); i++ ) {
        if ( !enables_node[i].getBool(enables_attr[i].c_str()) ) {
            enabled = false;
            break;
        }
    }

    bool debug = component_node.getBool("debug");

    if ( Ts > 0.0) {
        if ( debug ) printf("Updating %s Ts = %.2f", get_name().c_str(), Ts );

        double y_n = 0.0;
	y_n = input_node.getDouble(input_attr.c_str());

        double r_n = 0.0;
	if ( ref_value != "" ) {
	    r_n = atof(ref_value.c_str());
	} else {
            r_n = ref_node.getDouble(ref_attr.c_str());
	}
                      
        if ( debug ) printf("  input = %.3f ref = %.3f\n", y_n, r_n );

        // Calculates proportional error:
        ep_n = config_node.getDouble("beta") * (r_n - y_n);
        if ( debug ) {
	    printf( "  ep_n = %.3f", ep_n);
	    printf( "  ep_n_1 = %.3f", ep_n_1);
	}

        // Calculates error:
        e_n = r_n - y_n;
        if ( debug ) printf( " e_n = %.3f", e_n);

        // Calculates derivate error:
        ed_n = config_node.getDouble("gamma") * r_n - y_n;
        if ( debug ) printf(" ed_n = %.3f", ed_n);

	double Td = config_node.getDouble("Td");
        if ( Td > 0.0 ) {
            // Calculates filter time:
            Tf = config_node.getDouble("alpha") * Td;
            if ( debug ) printf(" Tf = %.3f", Tf);

            // Filters the derivate error:
            edf_n = edf_n_1 / (Ts/Tf + 1)
                + ed_n * (Ts/Tf) / (Ts/Tf + 1);
            if ( debug ) printf(" edf_n = %.3f", edf_n);
        } else {
            edf_n = ed_n;
        }

        // Calculates the incremental output:
	double Ti = config_node.getDouble("Ti");
	double Kp = config_node.getDouble("Kp");
        if ( Ti > 0.0 ) {
            delta_u_n = Kp * ( (ep_n - ep_n_1)
                               + ((Ts/Ti) * e_n)
                               + ((Td/Ts) * (edf_n - 2*edf_n_1 + edf_n_2)) );
        }

        if ( debug ) {
	    printf(" delta_u_n = %.3f\n", delta_u_n);
            printf("P: %.3f  I: %.3f  D:%.3f\n",
		   Kp * (ep_n - ep_n_1),
		   Kp * ((Ts/Ti) * e_n),
		   Kp * ((Td/Ts) * (edf_n - 2*edf_n_1 + edf_n_2)));
        }

        // Integrator anti-windup logic:
	double u_min = config_node.getDouble("u_min");
	double u_max = config_node.getDouble("u_max");
        if ( delta_u_n > (u_max - u_n_1) ) {
            delta_u_n = u_max - u_n_1;
            if ( debug ) printf(" max saturation\n");
        } else if ( delta_u_n < (u_min - u_n_1) ) {
            delta_u_n = u_min - u_n_1;
            if ( debug ) printf(" min saturation\n");
        }

        // Calculates absolute output:
        u_n = u_n_1 + delta_u_n;
        if ( debug ) printf("  output = %.3f\n", u_n);

        // Updates indexed values;
        u_n_1   = u_n;
        ep_n_1  = ep_n;
        edf_n_2 = edf_n_1;
        edf_n_1 = edf_n;
    }

    if ( enabled ) {
	// Copy the result to the output node(s)
	for ( unsigned int i = 0; i < output_node.size(); i++ ) {
	    output_node[i].setDouble( output_attr[i].c_str(), u_n );
	}
    } else if ( output_node.size() > 0 ) {
	// Mirror the output value while we are not enabled so there
	// is less of a continuity break when this module is enabled

	// pull output value from the corresponding property tree value
	u_n = output_node[0].getDouble(output_attr[0].c_str());
	// and clip
	double u_min = config_node.getDouble("u_min");
	double u_max = config_node.getDouble("u_max");
 	if ( u_n < u_min ) { u_n = u_min; }
	if ( u_n > u_max ) { u_n = u_max; }
	u_n_1 = u_n;
    }
}
