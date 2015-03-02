/**
 * \file: gnss_interface.h
 *
 * C++/Property aware interface for GNSS/ADNS 15-state kalman filter algorithm
 *
 * Copyright (C) 2009 - Curtis L. Olson
 *
 */


#ifndef _UGEAR_GNSS_ADNS_INTERFACE_H
#define _UGEAR_GNSS_ADNS_INTERFACE_H


#include <string>

#include "props/props.hxx"

using std::string;



int uggnss_adns_init( string rootname, SGPropertyNode *config );
bool uggnss_adns_update();
int uggnss_adns_close();


#endif // _UGEAR_GNSS_ADNS_INTERFACE_H
