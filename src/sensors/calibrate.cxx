/**
 * \file: calibrate.cxx
 *
 * Callibration helper class
 *
 * Copyright (C) 2015 - Curtis L. Olson curtolson@flightgear.org
 *
 */

#include <stdio.h>

#include "util/strutils.hxx"

#include "calibrate.hxx"

// set parameters to default zero bias and 1.0 scaling factor
void UGCalibrate::defaults()
{
    _min_temp = 27.0;
    _max_temp = 27.0;
    
    bias[0] = 0.0;
    bias[1] = 0.0;
    bias[2] = 0.0;
    scale[0] = 0.0;
    scale[1] = 0.0;
    scale[2] = 1.0;
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
void UGCalibrate::init( SGPropertyNode *config, float min_temp, float max_temp )
{
    defaults();

    _min_temp = min_temp;
    _max_temp = max_temp;
    
    SGPropertyNode *node = NULL;
    
    node = config->getChild("bias");
    if ( node != NULL ) {
	string bias_str = node->getString();
	vector<string> tokens = split( bias_str );
	if ( tokens.size() == 1 ) {
	    // constant bias
	    bias[0] = 0.0;
	    bias[1] = 0.0;
	    bias[2] = atof( tokens[0].c_str() );
	} else if ( tokens.size() == 3 ) {
	    // 2nd degree polynomial fit
	    bias[0] = atof( tokens[0].c_str() );
	    bias[1] = atof( tokens[1].c_str() );
	    bias[2] = atof( tokens[2].c_str() );
	} else {
	    // error, set zero biase
	    bias[0] = 0.0;
	    bias[1] = 0.0;
	    bias[2] = 0.0;
	}
    }
    node = config->getChild("scale");
    if ( node != NULL ) {
	string scale_str = node->getString();
	vector<string> tokens = split( scale_str );
	if ( tokens.size() == 1 ) {
	    // constant scale
	    scale[0] = 0.0;
	    scale[1] = 0.0;
	    scale[2] = atof( tokens[0].c_str() );
	} else if ( tokens.size() == 3 ) {
	    // 2nd degree polynomial fit
	    scale[0] = atof( tokens[0].c_str() );
	    scale[1] = atof( tokens[1].c_str() );
	    scale[2] = atof( tokens[2].c_str() );
	} else {
	    // error, set 1.0 scale
	    scale[0] = 0.0;
	    scale[1] = 0.0;
	    scale[2] = 1.0;
	}
    }
    // printf("bias = %.6f %.6f %.6f, scale = %.4f\n",
    //        bias[0], bias[1], bias[2], scale);
}


