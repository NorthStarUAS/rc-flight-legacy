/**
 * \file: gps_mgr.h
 *
 * Front end management interface for reading GPS data.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@flightgear.org
 *
 */

#pragma once

class gps_helper_t {
public:
    void init();
    void update();
    double gps_age();           // seconds
    
private:
    pyPropertyNode gps_node;
    int gps_state = 0;
    double gps_acq_time = 0.0;
    double last_time = 0.0;

    void compute_magvar();
};

extern gps_helper_t gps_helper;
