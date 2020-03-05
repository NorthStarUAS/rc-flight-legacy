/**
 * \file: gps_ublox6.h
 *
 * u-blox 6 protocol driver
 *
 * Copyright Curt Olson curtolson@flightgear.org
 *
 */

#pragma once

#include <pyprops.h>

#include "include/globaldefs.h"

void gps_ublox6_init( string output_path, pyPropertyNode *config );
bool gps_ublox6_update();
void gps_ublox6_close();
