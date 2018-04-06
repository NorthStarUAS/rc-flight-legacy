/**
 * \file: raven2.hxx
 *
 * Driver for the Bolder Flight Systems Raven sensor/pwm module
 *
 * Copyright (C) 2016 - Curtis L. Olson - curtolson@flightgear.org
 *
 */

#pragma once

#include "python/pyprops.hxx"

#include <string>
using std::string;

#include "include/globaldefs.h"


void raven2_airdata_init( string output_path, pyPropertyNode *config );
bool raven2_airdata_update();
void raven2_airdata_close();

bool raven2_act_init( pyPropertyNode *config );
bool raven2_act_update();
void raven2_act_close();
