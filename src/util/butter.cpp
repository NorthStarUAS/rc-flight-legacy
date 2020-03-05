#include <math.h>
#include <stdio.h>

#include "butter.h"

ButterworthFilter::ButterworthFilter(int order, int samplerate, double cutoff)
{
    n = order / 2;
    double a = tan( M_PI * cutoff / samplerate );
    double a2 = a*a;
    A = new double[n];
    d1 = new double[n];
    d2 = new double[n];
    w0 = new double[n];
    w1 = new double[n];
    w2 = new double[n];

    // generate coefficients
    for ( int i = 0; i < n; ++i ) {
        double r = sin(M_PI*(2.0*i+1.0)/(4.0*n));
        double s = a2 + 2.0*a*r + 1.0;
        A[i] = a2/s;
        d1[i] = 2.0*(1-a2)/s;
        d2[i] = -(a2 - 2.0*a*r + 1.0)/s;
    }
}

ButterworthFilter::~ButterworthFilter() {
    delete[] A;
    delete[] d1;
    delete[] d2;
    delete[] w0;
    delete[] w1;
    delete[] w2;   
}

double ButterworthFilter::update( double raw_value ) {
    double x = raw_value;
    for ( int i = 0; i < n; ++i ) {
        w0[i] = d1[i]*w1[i] + d2[i]*w2[i] + x;
        x = A[i]*(w0[i] + 2.0*w1[i] + w2[i]);
        w2[i] = w1[i];
        w1[i] = w0[i];
    }
    return x;
}
