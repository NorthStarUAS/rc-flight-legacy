//
// aura_interface.hxx -- C++/Property aware interface for GNSS/ADNS 15-state
//                       kalman filter algorithm
//

#ifndef _AURA_NAV_EKF15_INTERFACE_HXX
#define _AURA_NAV_EFK15_INTERFACE_HXX


#include "python/pyprops.hxx"

#include <string>
using std::string;

void nav_ekf15_init( string output_path, pyPropertyNode *config );
bool nav_ekf15_update();
void nav_ekf15_close();


#endif // _AURA_NAV_EKF15_INTERFACE_HXX
