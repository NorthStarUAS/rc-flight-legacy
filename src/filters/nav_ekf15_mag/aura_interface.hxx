//
// aura_interface.hxx -- C++/Property aware interface for GNSS/ADNS 15-state
//                       kalman filter algorithm
//

#pragma once

#include <pyprops.hxx>

#include <string>
using std::string;

void nav_ekf15_mag_init( string output_path, pyPropertyNode *config );
void nav_ekf15_mag_reset();
bool nav_ekf15_mag_update();
void nav_ekf15_mag_close();
