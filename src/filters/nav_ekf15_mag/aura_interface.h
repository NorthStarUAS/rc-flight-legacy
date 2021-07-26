//
// aura_interface.h -- C++/Property aware interface for GNSS/ADNS 15-state
//                     kalman filter algorithm
//

#pragma once

#include <props2.h>

#include <string>
using std::string;

void nav_ekf15_mag_init( string output_path, PropertyNode *config );
void nav_ekf15_mag_reset();
bool nav_ekf15_mag_update();
void nav_ekf15_mag_close();
