/**
 * \file: gps_mgr.h
 *
 * Front end management interface for reading GPS data.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@flightgear.org
 *
 */

#pragma once

void GPS_init();
bool GPS_update();

// return gps data age in seconds
double GPS_age();
