/**
 * \file: raven.hxx
 *
 * Driver for the Bolder Flight Systems Raven sensor/pwm module
 *
 * Copyright (C) 2016 - Curtis L. Olson - curtolson@flightgear.org
 *
 */

#ifndef _AURA_BFS_RAVEN_HXX
#define _AURA_BFS_RAVEN_HXX


#include "python/pyprops.hxx"

#include <string>
using std::string;

#include "include/globaldefs.h"


void raven_airdata_init( string output_path, pyPropertyNode *config );
bool raven_airdata_update();
void raven_airdata_close();

bool raven_act_init( string output_path, pyPropertyNode *config );
bool raven_act_update();
void raven_act_close();

#endif // _AURA_BFS_RAVEN_HXX
