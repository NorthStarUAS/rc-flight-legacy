/**
 * \file: cal_temp.h
 *

 * Temp calibration helper class.  This rolls two concepts together into a
 * single class:

 * a temperature bias fit function, currently a 2nd degree polynomal
 * where the parameters are the coefficients:
 * y = bias[0]*x*x + bias[1]*x + bias[2]

 * a scale parameter (maybe someday a set of parameters like bias)

 * the calibrate() function returns a calibrated value given a raw
 * sensor value and a temperature: cal = (raw - bias) * scale where
 * bias is a function of temp.

 *
 * Copyright (C) 2015 - Curtis L. Olson curtolson@flightgear.org
 *
 */

#pragma once

#include <pyprops.h>
#include "util/poly1d.h"

class AuraCalTemp {

private:

    float _min_temp;		// temp (C)
    float _max_temp;		// temp (C)
    
    AuraPoly1d bias;
    AuraPoly1d scale;

    void defaults();

public:

    AuraCalTemp();
    ~AuraCalTemp();

    void init( pyPropertyNode *config, float min_temp, float max_temp );

    inline float get_bias( float temp )  {
	if ( temp < _min_temp ) { temp = _min_temp; }
	if ( temp > _max_temp ) { temp = _max_temp; }
	return bias.eval(temp);
    }

    inline float get_scale( float temp )  {
	if ( temp < _min_temp ) { temp = _min_temp; }
	if ( temp > _max_temp ) { temp = _max_temp; }
	return scale.eval(temp);
    }
    
    inline float calibrate( float x, float temp ) {
	float b = get_bias( temp );
	float s = get_scale( temp );
	// printf("sensor @ %.1f: %.3f -> %.3f\n", temp, x, (x - bias) * scale);
	return (x - b) * s;
    }
};
