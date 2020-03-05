// a class to implement a simple linear fit using low pass filtered
// internal terms so it should track changes over time and slowly
// forget history

// this class requires a hardwired dt set at initialization.  A
// variable dt can lead to incorrect output.  Hopefully the calling
// layer can manage to call the update() function at close to the
// provided dt.

#pragma once

class LinearFitFilter {
    
private:
    
    double _time_factor;
    double _dt;
    int n;
    
    double sum_x;
    double sum_y;
    double sum_x2;
    double sum_y2;
    double sum_xy;

    // the current fit estimate
    double a0;
    double a1;

public:
    
    LinearFitFilter( double time_factor, double dt );
    ~LinearFitFilter();
    void init( double value );
    void update( double x, double y );
    void reset();
    inline double get_value( double x ) {
	return a1*x + a0;
    }
};
