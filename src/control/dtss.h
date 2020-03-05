// dtss.h - a discrete time state space controller
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


#pragma once

#include <string>
using std::string;

#include <eigen3/Eigen/Core>
using namespace Eigen;

#include "component.h"


typedef Matrix<double, Dynamic, Dynamic> MatrixXd;
typedef Matrix<double, Dynamic, 1> VectorXd;


class AuraDTSS : public APComponent {

private:

    unsigned int nx, nz, nu;
    bool do_reset;

    VectorXd x, z, u;
    MatrixXd A, B, C, D;
    MatrixXd M, S, T, F, G;
    
    vector <pyPropertyNode> inputs_node;
    vector <string> inputs_attr;
    VectorXd z_trim;
    
    vector <pyPropertyNode> outputs_node;
    vector <string> outputs_attr;

    vector <double> u_min;
    vector <double> u_max;
    vector <double> u_trim;
    
public:

    AuraDTSS( string config_path );
    ~AuraDTSS() {}

    void reset();
    void update( double dt );
};
