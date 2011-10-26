/**
 * \file: umn_interface.h
 *
 * C++/Property aware interface for UMN ADNS algorithm
 *
 * Copyright (C) 2009 - Curtis L. Olson
 *
 * $Id: umn_interface.h,v 1.1 2009/05/15 17:04:56 curt Exp $
 */


#ifndef _UGEAR_ADNS_UMN_INTERFACE_H
#define _UGEAR_ADNS_UMN_INTERFACE_H


#include <string>

#include "props/props.hxx"

using std::string;



int ugumn_adns_init( string rootname, SGPropertyNode *config );
bool ugumn_adns_update();
int ugumn_adns_close();


#endif // _UGEAR_ADNS_UMN_INTERFACE_H
