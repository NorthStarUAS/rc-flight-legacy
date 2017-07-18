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


#include <string>
using std::string;

#include <eigen3/Eigen/Core>
using namespace Eigen;

#include "component.hxx"


typedef Matrix<double, Dynamic, Dynamic> MatrixXd;
typedef Matrix<double, Dynamic, 1> VectorXd;


class AuraDTSS : public APComponent {

private:

    int nx, nz, nu;
    bool first_time;

    VectorXd x, z, z_prev, u;
    MatrixXd A, B, C, D;
    
    // post functions for output
    bool clamp;

    vector <pyPropertyNode> inputs_node;
    vector <string> inputs_attr;
    
    vector <pyPropertyNode> outputs_node;
    vector <string> outputs_attr;

public:

    AuraDTSS( string config_path );
    ~AuraDTSS() {}

    void update( double dt );
};


