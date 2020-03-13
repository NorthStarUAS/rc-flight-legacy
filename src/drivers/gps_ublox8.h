/**
 * \file: gps_ublox8.h
 *
 * u-blox 7 protocol driver
 *
 * Copyright Curt Olson curtolson@flightgear.org
 *
 */

#pragma once

#include <pyprops.h>
#include "include/globaldefs.h"

void gps_ublox8_init( string output_path, pyPropertyNode *config );
bool gps_ublox8_update();
void gps_ublox8_close();
