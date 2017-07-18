// dtss.hxx - a discrete time state space controller
//
// Written by Curtis Olson & Raghu Venkataraman, started July 2017.
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

#include <iostream>

#include "python/pyprops.hxx"

#include "dtss.hxx"


AuraDTSS::AuraDTSS( string config_path ):
    nx(1),
    nz(1),
    nu(1),
    first_time(true),
    clamp( false )
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

    vector <string> children;
    
    // inputs
    node = component_node.getChild( "inputs", true );
    children = node.getChildren();
    nz = children.size();
    printf("dtss: %d input(s)\n", children.size());
    for ( unsigned int i = 0; i < children.size(); ++i ) {
	if ( children[i].substr(0,4) == "prop" ) {
	    string input_prop = node.getString(children[i].c_str());
            printf("  %s\n", input_prop.c_str());
	    pos = input_prop.rfind("/");
	    if ( pos != string::npos ) {
		string path = input_prop.substr(0, pos);
		string attr = input_prop.substr(pos+1);
		pyPropertyNode inode = pyGetNode( path, true );
		inputs_node.push_back( inode );
		inputs_attr.push_back( attr );
	    } else {
		printf("WARNING: requested bad output path: %s\n",
		       input_prop.c_str());
	    }
	} else {
	    printf("WARNING: unknown tag in output section: %s\n",
		   children[i].c_str());
	}
    }

    // outputs
    node = component_node.getChild( "outputs", true );
    children = node.getChildren();
    nu = children.size();
    printf("dtss: %d output(s)\n", children.size());
    for ( unsigned int i = 0; i < children.size(); ++i ) {
	if ( children[i].substr(0,4) == "prop" ) {
	    string output_prop = node.getString(children[i].c_str());
            printf("  %s\n", output_prop.c_str());
	    pos = output_prop.rfind("/");
	    if ( pos != string::npos ) {
		string path = output_prop.substr(0, pos);
		string attr = output_prop.substr(pos+1);
		pyPropertyNode onode = pyGetNode( path, true );
		outputs_node.push_back( onode );
		outputs_attr.push_back( attr );
	    } else {
		printf("WARNING: requested bad output path: %s\n",
		       output_prop.c_str());
	    }
	} else {
	    printf("WARNING: unknown tag in output section: %s\n",
		   children[i].c_str());
	}
    }

    int len;
    
    // A matrix
    len = component_node.getLen("A");
    int nx = round(sqrt(len));
    if ( nx * nx != len ) {
        printf("A improperly sized, len = %d\n", len);
        nx = (int)sqrt(len);
    }
    A = MatrixXd(nx, nx);
    for ( unsigned int r = 0; r < nx; ++r ) {
        for ( unsigned int c = 0; c < nx; ++c ) {
            A(r,c) = component_node.getDouble("A", r*nx + c);
        }
    }
    std::cout << A << std::endl;
    
    // B matrix
    len = component_node.getLen("B");
    if ( len != nx * nz ) {
        printf("B improperly sized, len = %d\n", len);
    }
    B = MatrixXd(nx, nz);
    for ( unsigned int r = 0; r < nx; ++r ) {
        for ( unsigned int c = 0; c < nz; ++c ) {
            B(r,c) = component_node.getDouble("B", r*nz + c);
        }
    }
    std::cout << B << std::endl;
    
    // C matrix
    len = component_node.getLen("C");
    if ( len != nu * nx ) {
        printf("C improperly sized, len = %d\n", len);
    }
    C = MatrixXd(nu, nx);
    for ( unsigned int r = 0; r < nu; ++r ) {
        for ( unsigned int c = 0; c < nx; ++c ) {
            C(r,c) = component_node.getDouble("C", r*nx + c);
        }
    }
    std::cout << C << std::endl;
    
    // D matrix
    len = component_node.getLen("D");
    if ( len != nu * nz ) {
        printf("D improperly sized, len = %d\n", len);
    }
    D = MatrixXd(nu, nz);
    for ( unsigned int r = 0; r < nu; ++r ) {
        for ( unsigned int c = 0; c < nz; ++c ) {
            D(r,c) = component_node.getDouble("D", r*nz + c);
        }
    }
    std::cout << D << std::endl;

    // initial state is zero
    x = VectorXd(nx);
    z = VectorXd(nz);
    z_prev = VectorXd(nz);
    u = VectorXd(nu);
    x.setZero();
    z.setZero();
    z_prev.setZero();
    u.setZero();
    
    // config
    config_node = component_node.getChild( "config", true );
}


void AuraDTSS::update( double dt ) {
    if (!enable_node.isNull() && enable_node.getString(enable_attr.c_str()) == enable_value) {
	enabled = true;
    } else {
	enabled = false;
    }

    bool debug = component_node.getBool("debug");
    if ( debug ) printf("Updating %s\n", get_name().c_str());

    // assemble input vector
    for ( unsigned int i = 0; i < nz; ++i ) {
        z(i) = inputs_node[i].getDouble(inputs_attr[i].c_str());
    }
    
    if ( first_time ) {
        first_time = false;
        u = C*x + D*z;
    } else {
        x = A*x + B*z_prev;
        u = C*x + D*z;
    }
    z_prev = z;

    std::cout << "u: " << u << std::endl;
    
    // y_n = input_node.getDouble(input_attr.c_str());

    // double r_n = 0.0;
    // if ( ref_value != "" ) {
    //     // printf("nonzero ref_value\n");
    //     r_n = atof(ref_value.c_str());
    // } else {
    //     r_n = ref_node.getDouble(ref_attr.c_str());
    // }
                      
    // double error = r_n - y_n;
    // if ( debug ) printf("input = %.3f reference = %.3f error = %.3f\n",
    //     		y_n, r_n, error);

    // double u_trim = config_node.getDouble("u_trim");
    // double u_min = config_node.getDouble("u_min");
    // double u_max = config_node.getDouble("u_max");

    // double Kp = config_node.getDouble("Kp");
    // double Ti = config_node.getDouble("Ti");
    // double Td = config_node.getDouble("Td");
    // double Ki = 0.0;
    // if ( Ti > 0.0001 ) {
    //     Ki = Kp / Ti;
    // }
    // double Kd = Kp * Td;

    // // proportional term
    // double pterm = Kp * error + u_trim;

    // // integral term
    // iterm += Ki * error * dt;
    
    // // derivative term: observe that dError/dt = -dInput/dt (except
    // // when the setpoint changes (which we don't want to react to
    // // anyway.)  This approach avoids "derivative kick" when the set
    // // point changes.
    // double dy = y_n - y_n_1;
    // y_n_1 = y_n;
    // double dterm = Kd * -dy / dt;

    // double output = pterm + iterm + dterm;
    // if ( output < u_min ) {
    //     iterm += u_min - output;
    //     output = u_min;
    // }
    // if ( output > u_max ) {
    //     iterm -= output - u_max;
    //     output = u_max;
    // }

    // if ( debug ) printf("pterm = %.3f iterm = %.3f\n",
    //     		pterm, iterm);
    // if ( debug ) printf("clamped output = %.3f\n", output);

    // if ( enabled ) {
    //     // Copy the result to the output node(s)
    //     for ( unsigned int i = 0; i < output_node.size(); i++ ) {
    //         output_node[i].setDouble( output_attr[i].c_str(), output );
    //     }
    // } else {
    //     // back compute an iterm that will produce zero initial
    //     // transient when activating this component
    //     double u_n = output_node[0].getDouble(output_attr[0].c_str());
    //     // and clip
    //     double u_min = config_node.getDouble("u_min");
    //     double u_max = config_node.getDouble("u_max");
    //     if ( u_n < u_min ) { u_n = u_min; }
    //     if ( u_n > u_max ) { u_n = u_max; }
    //     iterm = u_n - pterm;
    // }
}


