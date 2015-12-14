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

#ifndef _AURA_CALIBRATE_HXX
#define _AURA_CALIBRATE_HXX

#include "props/props.hxx"


class UGCalibrate {

private:

    float _min_temp;		// temp (C)
    float _max_temp;		// temp (C)
    
    float bias[3];
    float scale[3];

    void defaults();

public:

    UGCalibrate();
    ~UGCalibrate();

    void init( SGPropertyNode *config, float min_temp, float max_temp );

    inline float eval_bias( float temp )  {
	if ( temp < _min_temp ) { temp = _min_temp; }
	if ( temp > _max_temp ) { temp = _max_temp; }
	return bias[0]*temp*temp + bias[1]*temp + bias[2];
    }

    inline float eval_scale( float temp )  {
	if ( temp < _min_temp ) { temp = _min_temp; }
	if ( temp > _max_temp ) { temp = _max_temp; }
	return scale[0]*temp*temp + scale[1]*temp + scale[2];
    }
    
    inline float calibrate( float x, float temp ) {
	float b = eval_bias( temp );
	float s = eval_scale( temp );
	// printf("sensor @ %.1f: %.3f -> %.3f\n", temp, x, (x - bias) * scale);
	return (x - b) * s;
    }
};

#endif // _AURA_CALIBRATE_HXX
