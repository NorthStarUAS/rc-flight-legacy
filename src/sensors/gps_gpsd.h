/**
 * \file: gpsd.h
 *
 * Read gps data from the gpsd driver (provides data via a socket
 * connection on port 2947)
 *
 * Copyright Curt Olson curtolson@flightgear.org
 *
 */

#pragma once

#include <pyprops.h>

#include "include/globaldefs.h"

void gpsd_init( string output_path, pyPropertyNode *config );
bool gpsd_get_gps();
void gpsd_close();
