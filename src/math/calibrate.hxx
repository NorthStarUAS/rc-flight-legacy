/**
 * \file: calibrate.hxx
 *

 * Callibration helper class.  This rolls two concepts together into a
 * single class:

 * a temperature bias fit function, currently a 2nd degree polynomal
 * where the parameters are the coefficients:
 * y = bias[0]*x*x + bias[1]*x + bias[2]

 * a scale parameter (maybe someday a set of parameters like bias)

 * the calibrate() function returns a calibrated value given a raw
 * sensor value and a temperature: cal = (raw - bias) * scale where
 * bias is a function of temp.

 *
 * Copyright (C) 2015 - Curtis L. Olson curtolson@gmail.com
 *
 */

#ifndef _UGEAR_CALIBRATE_HXX
#define _UGEAR_CALIBRATE_HXX

#include "props/props.hxx"


class UGCalibrate {

private:
    
    float bias[3];
    float scale[3];

    void defaults();

public:

    UGCalibrate();
    ~UGCalibrate();

    void init( SGPropertyNode *config );

    inline float eval_bias( float x )  {
	return bias[0]*x*x + bias[1]*x + bias[2];
    }

    inline float eval_scale( float x )  {
	return scale[0]*x*x + scale[1]*x + scale[2];
    }
    
    inline float calibrate( float x, float temp ) {
	float b = eval_bias( temp );
	float s = eval_scale( temp );
	// printf("sensor @ %.1f: %.3f -> %.3f\n", temp, x, (x - bias) * scale);
	return (x - b) * s;
    }
};

#endif // _UGEAR_CALIBRATE_HXX
