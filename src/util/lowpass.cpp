#include <stdio.h>

#include "lowpass.h"

LowPassFilter::LowPassFilter()
{
    set_time_factor( 1.0 );
    filter_value = 0.0;
    inited = false;
}

LowPassFilter::LowPassFilter( double time_factor )
{
    set_time_factor( time_factor );
    filter_value = 0.0;
    inited = false;
}

LowPassFilter::LowPassFilter( double time_factor, double init )
{
    set_time_factor( time_factor );
    filter_value = init;
    inited = true;
}

LowPassFilter::~LowPassFilter() {}

double LowPassFilter::update( double value, double dt ) {
    if ( ! inited ) {
	filter_value = value;
	inited = true;
    }
    
    // Weight factor (wf): the actual low pass filter value for the
    // current dt.
    double weight_factor;
    if ( _time_factor > 0.0 ) {
	weight_factor = dt / _time_factor;
    } else {
	weight_factor = 1.0;
    }
    
    // The greater the weight, the noisier the filter, but the faster
    // it converges.  Must be > 0.0 or value will never converge.  Max
    // weight is 1.0 which means we just use the raw input value with
    // no filtering.  Min weight is 0.0 which means we do not change
    // the filtered value (but might drift over time with numerical
    // rounding.)
    if ( weight_factor < 0.0 ) { weight_factor = 0.0; }
    if ( weight_factor > 1.0 ) { weight_factor = 1.0; }
    filter_value = (1.0 - weight_factor) * filter_value
	+ weight_factor * value;
    return filter_value;
}
