/**
 * \file: calibrate.cxx
 *
 * Callibration helper class
 *
 * Copyright (C) 2015 - Curtis L. Olson curtolson@flightgear.org
 *
 */

#include "python/pyprops.hxx"

#include <stdio.h>

#include "util/strutils.hxx"

#include "calibrate.hxx"

// set parameters to default zero bias and 1.0 scaling factor
void UGCalibrate::defaults()
{
    _min_temp = 27.0;
    _max_temp = 27.0;

    vector<double> bias_coeffs {0.0, 0.0, 0.0};
    vector<double> scale_coeffs {0.0, 0.0, 1.0};
    bias = AuraPoly1d(bias_coeffs);
    scale = AuraPoly1d(scale_coeffs);
}


UGCalibrate::UGCalibrate()
{
    defaults();
}


UGCalibrate::~UGCalibrate()
{
    // nothing to do
}


// load parameters from specified property subtree
void UGCalibrate::init( pyPropertyNode *config, float min_temp, float max_temp )
{
    defaults();

    _min_temp = min_temp;
    _max_temp = max_temp;
    
    if ( config->hasChild("bias") ) {
	string bias_str = config->getString("bias");
	vector<string> tokens = split( bias_str );
	vector<double> bias_coeffs;
	for ( unsigned int i = 0; i < tokens.size(); i++ ) {
	    bias_coeffs.push_back( atof(tokens[i].c_str()) );
	}
	bias = AuraPoly1d(bias_coeffs);
    }
    if ( config->hasChild("scale") ) {
	string scale_str = config->getString("scale");
	vector<string> tokens = split( scale_str );
	vector<double> scale_coeffs;
	for ( unsigned int i = 0; i < tokens.size(); i++ ) {
	    scale_coeffs.push_back( atof(tokens[i].c_str()) );
	}
	scale = AuraPoly1d(scale_coeffs);
    }
    // printf("bias = %.6f %.6f %.6f, scale = %.4f\n",
    //        bias[0], bias[1], bias[2], scale);
}


