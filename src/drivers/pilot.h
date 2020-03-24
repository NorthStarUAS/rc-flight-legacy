/**
 * \file: pilot.h
 *
 * Pilot input helper
 *
 * Copyright (C) 2010-2020 - Curtis L. Olson curtolson@flightgear.org
 *
 */

#pragma once

class pilot_helper_t {
public:
    void init();
    void update();

private:
    pyPropertyNode pilot_node;
    pyPropertyNode flight_node;
    pyPropertyNode engine_node;
    pyPropertyNode ap_node;
};

extern pilot_helper_t pilot_helper;
