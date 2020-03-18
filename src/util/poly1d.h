/**
 * \file: poly1d.h
 *
 * Polygon evalutation class.  (Inspired by the poly1d python module.)
 *
 * Copyright (C) 2016 - Curtis L. Olson curtolson@flightgear.org
 *
 */

#pragma once

#include <string>
#include <vector>
using std::string;
using std::vector;

#include <eigen3/Eigen/Core>
using namespace Eigen;

#include "util/strutils.h"

class AuraPoly1d {

private:

    vector<double> _coeffs;

public:

    AuraPoly1d() {
	_coeffs = vector<double>(2,0);
        _coeffs[0] = 1.0; // y = 1*x + 0; y = x
    }
    AuraPoly1d( vector<double> coeffs ) {
	if ( coeffs.size() ) {
	    _coeffs = coeffs;
	} else {
	    _coeffs = vector<double>(2,0);
            _coeffs[0] = 1.0; // y = 1*x + 0; y = x
	}
    }
    
    ~AuraPoly1d() { }

    inline double eval( double x )  {
	unsigned int size = _coeffs.size();
	double exp = 1.0;
	double sum = 0.0;	   
	for ( int i = size - 1; i >= 0; i-- ) {
	    sum += _coeffs[i]*exp;
	    exp *= x;
	}
	return sum;
    }

    inline void print() {
	unsigned int size = _coeffs.size();
	for ( int i = size - 1; i >= 0; i-- ) {
	    printf("(%d) %.8f, ", i, _coeffs[i]);
	}
	printf("\n");
    }
};
