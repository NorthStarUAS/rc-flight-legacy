//
// aura_interface.hxx -- C++/Property aware interface for GNSS/ADNS 15-state
//                       kalman filter algorithm
//

#ifndef _AURA_NAV_EIGEN_MAG_INTERFACE_HXX
#define _AURA_NAV_EIGEN_MAG_INTERFACE_HXX


#include "python/pyprops.hxx"

#include <string>
using std::string;

void nav_eigen_mag_init( string output_path, pyPropertyNode *config );
bool nav_eigen_mag_update();
void nav_eigen_mag_close();


#endif // _AURA_NAV_EIGEN_MAG_INTERFACE_HXX
