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

double pika_update();

bool pika_imu_init( string output_path, pyPropertyNode *config );
bool pika_imu_update();
void pika_imu_close();

void pika_airdata_init( string output_path, pyPropertyNode *config );
bool pika_airdata_update();
void pika_airdata_close();

bool pika_act_init( pyPropertyNode *config );
bool pika_act_update();
void pika_act_close();

#endif // _AURA_BFS_RAVEN_HXX
