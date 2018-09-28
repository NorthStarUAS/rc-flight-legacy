/**
 * \file: cal_temp.cxx
 *
 * Calibration helper class
 *
 * Copyright (C) 2015 - Curtis L. Olson curtolson@flightgear.org
 *
 */

#include <pyprops.hxx>

#include <stdio.h>

#include "util/strutils.hxx"

#include "cal_temp.hxx"

// set parameters to default zero bias and 1.0 scaling factor
void AuraCalTemp::defaults()
{
    _min_temp = 27.0;
    _max_temp = 27.0;

    vector<double> bias_coeffs(3,0);
    vector<double> scale_coeffs(3,0);
    scale_coeffs[2] = 1.0;
    bias = AuraPoly1d(bias_coeffs);
    scale = AuraPoly1d(scale_coeffs);
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
void AuraCalTemp::init( pyPropertyNode *config, float min_temp, float max_temp )
{
    defaults();

    _min_temp = min_temp;
    _max_temp = max_temp;
    
    if ( config->hasChild("bias") ) {
	bias = AuraPoly1d(config->getString("bias"));
	bias.print();
    }
    if ( config->hasChild("scale") ) {
	scale = AuraPoly1d(config->getString("scale"));
	scale.print();
    }
    // printf("bias = %.6f %.6f %.6f, scale = %.4f\n",
    //        bias[0], bias[1], bias[2], scale);
}


