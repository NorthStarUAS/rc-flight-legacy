/**
 * \file: airdata_mgr.h
 *
 * Front end management interface for reading air data.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@flightgear.org
 *
 */

#pragma once

#include <props2.h>

#include "util/lowpass.h"

class airdata_helper_t {
public:
    void init(DocPointerWrapper d);
    void update();

private:
    PropertyNode airdata_node;
    PropertyNode sensors_node;
    PropertyNode pos_filter_node;
    PropertyNode pos_pressure_node;
    PropertyNode pos_combined_node;
    PropertyNode status_node;
    PropertyNode task_node;
    PropertyNode wind_node;
    PropertyNode vel_node;
    
    LowPassFilter pressure_alt_filt = LowPassFilter( 0.1 );
    LowPassFilter ground_alt_filt = LowPassFilter( 30.0 );
    LowPassFilter airspeed_filt = LowPassFilter( 0.1 );
    LowPassFilter Ps_filt_err = LowPassFilter( 300.0 );
    LowPassFilter climb_filt = LowPassFilter( 1.0 );

    bool airdata_calibrated = false;
    bool alt_error_calibrated = false;
    float pressure_alt_filt_last = 0.0;
    double last_time = 0.0;

    float true_alt_m = 0.0;
};

//extern airdata_helper_t airdata_helper;
