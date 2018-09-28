/**
 * \file: raven1.hxx
 *
 * Driver for the Bolder Flight Systems Raven sensor/pwm module
 *
 * Copyright (C) 2016 - Curtis L. Olson - curtolson@flightgear.org
 *
 */

#pragma once

#include <pyprops.hxx>

#include <string>
using std::string;

#include "include/globaldefs.h"

void raven1_airdata_init( string output_path, pyPropertyNode *config );
bool raven1_airdata_update();
void raven1_airdata_close();

bool raven1_act_init( pyPropertyNode *config );
bool raven1_act_update();
void raven1_act_close();
