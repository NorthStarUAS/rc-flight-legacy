/**
 * \file: actuators.h
 *
 * convert logical flight controls into physical actuator outputs
 *
 * Copyright (C) 2009-2020 - Curtis L. Olson curtolson@flightgear.org
 */

#pragma once

#include <props2.h>

class actuators_t {
private:
    PropertyNode flight_node;
    PropertyNode engine_node;
    PropertyNode pilot_node;
    PropertyNode act_node;
    PropertyNode ap_node;
    PropertyNode excite_node;
    
public:
    actuators_t() {}
    ~actuators_t() {}
    void init(SharedStateWrapper d);
    void update();
};
