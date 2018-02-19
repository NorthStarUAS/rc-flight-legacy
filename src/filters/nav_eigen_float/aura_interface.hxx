//
// aura_interface.hxx -- C++/Property aware interface for GNSS/ADNS 15-state
//                       kalman filter algorithm
//

#ifndef _AURA_NAV_EIGEN_FLOAT_INTERFACE_HXX
#define _AURA_NAV_EIGEN_FLOAT_INTERFACE_HXX


#include "python/pyprops.hxx"

#include <string>
using std::string;

void nav_eigen_float_init( string output_path, pyPropertyNode *config );
bool nav_eigen_float_update();
void nav_eigen_float_close();


#endif // _AURA_NAV_EIGEN_FLOAT_INTERFACE_HXX
