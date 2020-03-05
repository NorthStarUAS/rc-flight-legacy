// dtss.cpp - a discrete time state space controller
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

#include <pyprops.h>

#include "dtss.h"


AuraDTSS::AuraDTSS( string config_path ):
    nx(1),
    nz(1),
    nu(1),
    do_reset(true)
{
    size_t pos;
    unsigned int len;
    
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
    
    // inputs
    node = component_node.getChild( "inputs", true );
    children = node.getChildren();
    nz = children.size();
    printf("dtss: %ld input(s)\n", children.size());
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
		printf("WARNING: requested bad input path: %s\n",
		       input_prop.c_str());
	    }
	} else {
	    printf("WARNING: unknown tag in input section: %s\n",
		   children[i].c_str());
	}
    }

    // z_trim
    z_trim = VectorXd(nz);
    len = component_node.getLen("z_trim");
    if ( len != nz ) {
        printf("WARNING: wrong number of elements for z_trim vector: %d\n", len);
    } else {
        for ( unsigned int i = 0; i < len; ++i ) {
            z_trim(i) = component_node.getDouble("z_trim", i);
        }
    }
    
    // outputs
    node = component_node.getChild( "outputs", true );
    children = node.getChildren();
    nu = component_node.getLen( "outputs" );
    printf("dtss: %d output(s)\n", nu);
    for ( unsigned int i = 0; i < nu; ++i ) {
        pyPropertyNode child = component_node.getChild( "outputs", i, true );
        string output_prop = child.getString("prop");        
        pos = output_prop.rfind("/");
        double min = child.getDouble("u_min");  
        double max = child.getDouble("u_max");
        double trim = child.getDouble("u_trim");
        printf("  %s [%.2f, %.2f]\n", output_prop.c_str(), min, max);
        if ( pos != string::npos ) {
            string path = output_prop.substr(0, pos);
            string attr = output_prop.substr(pos+1);
            pyPropertyNode onode = pyGetNode( path, true );
            outputs_node.push_back( onode );
            outputs_attr.push_back( attr );
            u_min.push_back( min );
            u_max.push_back( max );
            u_trim.push_back( trim );
        } else {
            printf("WARNING: requested bad output path: %s\n",
                   output_prop.c_str());
        }
    }

    // A matrix
    len = component_node.getLen("A");
    nx = round(sqrt(len));
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
    u = VectorXd(nu);
    x.setZero();
    z.setZero();
    u.setZero();

    // allocate internally used matrices
    M = MatrixXd(nx + nz, nx + nz);
    S = MatrixXd(nx + nz, nx + nz);
    T = MatrixXd(nx + nz, nx + nz);
    F = MatrixXd(nx, nx);
    G = MatrixXd(nx, nz);

    // config
    config_node = component_node.getChild( "config", true );
}


void AuraDTSS::reset() {
    do_reset = true;
}


void AuraDTSS::update( double dt ) {
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

    // construct the M matrix
    M.topLeftCorner(nx,nx) = A * dt;
    M.topRightCorner(nx,nz) = B * dt;
    M.bottomRows(nz).setZero();

    // compute the S matrix = I + M + M^2/2 + M^3/6
    S.setIdentity();
    T = M;                      // M
    S += T;
    T *= M;                     // M^2
    S += T / 2.0;
    T *= M;                     // M^3
    S += T / 6.0;

    // extract the F & G matrices
    F = S.topLeftCorner(nx, nx);
    G = S.topRightCorner(nx, nz);

    // update states
    if ( do_reset ) {
        do_reset = false;
        x.setZero();
        for ( unsigned int i = 0; i < nz; ++i ) {
            z(i) = inputs_node[i].getDouble(inputs_attr[i].c_str());
        }
        u = C*x + D*(z - z_trim);
    } else {
        x = F*x + G*(z - z_trim);
        for ( unsigned int i = 0; i < nz; ++i ) {
            z(i) = inputs_node[i].getDouble(inputs_attr[i].c_str());
        }
        u = C*x + D*(z - z_trim);
    }

    if ( debug ) {
        std::cout << "z: " << z << std::endl;
        std::cout << "u: " << u << std::endl;
    }

    if ( !enabled ) {
        // this will force a reset when component becomes enabled
        do_reset = true;
    } else {
        // write outputs
        for ( unsigned int i = 0; i < nu; ++i ) {
            double value = u(i) + u_trim[i];
            if ( value < u_min[i] ) { value = u_min[i]; }
            if ( value > u_max[i] ) { value = u_max[i]; }
            outputs_node[i].setDouble( outputs_attr[i].c_str(), value );
        }
    }
}


