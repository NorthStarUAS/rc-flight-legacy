/**
 * \file: curt_adns.h
 *
 * Test bed for ADNS experimentation
 *
 * Copyright (C) 2009 - Curtis L. Olson
 *
 * $Id: umn_interface.h,v 1.1 2009/05/15 17:04:56 curt Exp $
 */


#pragma once

#include <pyprops.h>

int curt_adns_init( string output_path, pyPropertyNode *config );
int curt_adns_update( double dt );
int curt_adns_close();
