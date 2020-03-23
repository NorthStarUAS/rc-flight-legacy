/**
 * \file: actuators.h
 *
 * convert logical flight controls into physical actuator outputs
 *
 * Copyright (C) 2009-2020 - Curtis L. Olson curtolson@flightgear.org
 */

#pragma once

#include <pyprops.h>

class actuators_t {
private:
    pyPropertyNode flight_node;
    pyPropertyNode engine_node;
    pyPropertyNode pilot_node;
    pyPropertyNode act_node;
    pyPropertyNode ap_node;
    pyPropertyNode excite_node;
    
public:
    actuators_t() {}
    ~actuators_t() {}
    void init();
    void update();
};

extern actuators_t actuators;
