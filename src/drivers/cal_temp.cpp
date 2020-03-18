/**
 * \file: cal_temp.cpp
 *
 * Calibration helper class
 *
 * Copyright (C) 2015 - Curtis L. Olson curtolson@flightgear.org
 *
 */

#include <pyprops.h>

#include <stdio.h>

#include "util/strutils.h"

#include "cal_temp.h"

// set parameters to default zero bias and 1.0 scaling factor
void AuraCalTemp::defaults()
{
    _min_temp = 27.0;
    _max_temp = 27.0;

    vector<double> bias_coeffs(3,0);
    bias = AuraPoly1d(bias_coeffs);
}


AuraCalTemp::AuraCalTemp()
{
    defaults();
}


AuraCalTemp::~AuraCalTemp()
{
    // nothing to do
}


// load parameters from specified property subtree
void AuraCalTemp::init( vector<double> bias_coeffs, float min_temp, float max_temp )
{
    defaults();

    _min_temp = min_temp;
    _max_temp = max_temp;
    bias = AuraPoly1d(bias_coeffs);
    bias.print();
    // printf("bias = %.6f %.6f %.6f\n", bias[0], bias[1], bias[2]);
}


