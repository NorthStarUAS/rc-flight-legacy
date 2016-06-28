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

#include <string>
#include <vector>
using std::string;
using std::vector;

#include "util/strutils.hxx"

class AuraPoly1d {

private:

    vector<double> _coeffs;

public:

    AuraPoly1d() {
	_coeffs = vector<double> { 1.0, 0.0 }; // y = 1*x + 0; y = x
    }
    AuraPoly1d( vector<double> coeffs ) {
	if ( coeffs.size() ) {
	    _coeffs = coeffs;
	} else {
	    _coeffs = vector<double> { 1.0, 0.0 };
	}
    }
    AuraPoly1d( string coeffs_str ) {
	if ( coeffs_str != "" ) {
	    vector<string> tokens = split( coeffs_str );
	    _coeffs.clear();
	    for ( unsigned int i = 0; i < tokens.size(); i++ ) {
		_coeffs.push_back( atof(tokens[i].c_str()) );
	    }
	} else {
	    _coeffs = vector<double> { 1.0, 0.0 };
	}
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
