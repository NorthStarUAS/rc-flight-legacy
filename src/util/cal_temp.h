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

class AuraCalTemp {

private:

    float min_temp;		// temp (C)
    float max_temp;		// temp (C)
    float coeffs[3];            // polygon coeffs

    void defaults();

public:

    AuraCalTemp();
    ~AuraCalTemp();

    void init( float calib[3], float min, float max );

    inline float get_bias( float temp )  {
	if ( temp < min_temp ) { temp = min_temp; }
	if ( temp > max_temp ) { temp = max_temp; }
        return coeffs[0] + coeffs[1]*temp + coeffs[2]*temp*temp;
    }

    inline float calibrate( float x, float temp ) {
	// printf("sensor @ %.1f: %.3f -> %.3f\n", temp, x, x - bias);
	return x - get_bias(temp);
    }
};
