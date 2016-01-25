//
// umngnss_quat.h -- C++/Property aware interface for GNSS/ADNS 15-state
//                   kalman filter algorithm
//

#ifndef _AURA_UMN_GNSS_QUAT_H
#define _AURA_UMN_GNSS_QUAT_H


#include "python/pyprops.hxx"

#include <string>
using std::string;


void umngnss_quat_init( string output_path, pyPropertyNode *config );
bool umngnss_quat_update();
void umngnss_quat_close();


#endif // _AURA_UMN_GNSS_QUAT_H
