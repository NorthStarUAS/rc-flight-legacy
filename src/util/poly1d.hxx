/**
 * \file: poly1d.hxx
 *
 * Polygon evalutation class.  (Inspired by the poly1d python module.)
 *
 * Copyright (C) 2016 - Curtis L. Olson curtolson@flightgear.org
 *
 */

#ifndef _AURA_POLY1D_HXX
#define _AURA_POLY1D_HXX

#include <vector>
using std::vector;

class AuraPoly1d {

private:

    vector<double> _coeffs;

public:

    AuraPoly1d() { }
    AuraPoly1d( vector<double> coeffs ) {
	_coeffs = coeffs;
    }
    
    ~AuraPoly1d() { }

    inline double eval( double x )  {
	unsigned int size = _coeffs.size() - 1;
	double exp = 1.0;
	double sum = 0.0;	   
	for ( int i = size - 1; i >= 0; i-- ) {
	    sum += _coeffs[i]*exp;
	    exp *= x;
	}
	return sum;
    }
};

#endif // _AURA_POLY1D_HXX
