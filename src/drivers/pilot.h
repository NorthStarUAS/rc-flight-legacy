/**
 * \file: pilot.h
 *
 * Pilot input helper
 *
 * Copyright (C) 2010-2020 - Curtis L. Olson curtolson@flightgear.org
 *
 */

#pragma once

#include <props2.h>

class pilot_helper_t {
public:
    void init(DocPointerWrapper d);
    void update();

private:
    PropertyNode pilot_node;
    PropertyNode flight_node;
    PropertyNode engine_node;
    PropertyNode ap_node;
};

