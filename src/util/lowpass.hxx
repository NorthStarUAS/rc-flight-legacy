#ifndef _AURA_LOW_PASS_FILTER_HXX
#define _AURA_LOW_PASS_FILTER_HXX

// a class to implement a simple low pass filter

class LowPassFilter {
    
private:
    
    // Time factor (tf): length of time (sec) to low pass filter the
    // input over.  A time value of zero will result in the filter
    // output being equal to the raw input at each time step.
    float _time_factor;

    // the current filter value
    float filter_value;
    
public:
    
    LowPassFilter( float time_factor );
    ~LowPassFilter();
    void init( float value );
    float update( float value, float dt );
    float get_value();
};


#endif // _AURA_LOW_PASS_FILTER_HXX
