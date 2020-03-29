/**
 * \file: airdata_mgr.h
 *
 * Front end management interface for reading air data.
 *
 * Copyright (C) 2009 - Curtis L. Olson curtolson@flightgear.org
 *
 */

#pragma once

#include <pyprops.h>

#include "util/lowpass.h"

class airdata_helper_t {
public:
    void init();
    void update();

private:
    pyPropertyNode airdata_node;
    pyPropertyNode sensors_node;
    pyPropertyNode pos_filter_node;
    pyPropertyNode pos_pressure_node;
    pyPropertyNode pos_combined_node;
    pyPropertyNode status_node;
    pyPropertyNode task_node;
    pyPropertyNode wind_node;
    pyPropertyNode vel_node;
    
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

extern airdata_helper_t airdata_helper;
