//
// umngnss_euler.h -- C++/Property aware interface for GNSS/ADNS 15-state
//                    kalman filter algorithm
//

#ifndef _AURA_UMN_GNSS_EULER_H
#define _AURA_UMN_GNSS_EULER_H


#include <string>

#include "props/props.hxx"

using std::string;


int umngnss_euler_init( string rootname, SGPropertyNode *config );
bool umngnss_euler_update();
int umngnss_euler_close();


#endif // _AURA_UMN_GNSS_EULER_H
