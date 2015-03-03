//
// umngnss_quat.h -- C++/Property aware interface for GNSS/ADNS 15-state
//                   kalman filter algorithm
//

#ifndef _UGEAR_UMN_GNSS_QUAT_H
#define _UGEAR_UMN_GNSS_QUAT_H


#include <string>

#include "props/props.hxx"

using std::string;


void umngnss_quat_init( string rootname, SGPropertyNode *config );
bool umngnss_quat_update();
void umngnss_quat_close();


#endif // _UGEAR_UMN_GNSS_QUAT_H
