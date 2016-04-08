/**
 * \file: airdata_uart.hxx
 *
 * Airdata (UART) driver
 *
 * Copyright (C) 2016 - Curtis L. Olson - curtolson@flightgear.org
 *
 */

#ifndef _AURA_AIRDATA_UART_HXX
#define _AURA_AIRDATA_UART_HXX


#include "python/pyprops.hxx"

#include <string>
using std::string;

#include "include/globaldefs.h"


void airdata_uart_init( string output_path, pyPropertyNode *config );
bool airdata_uart_update();
void airdata_uart_close();


#endif // _AURA_AIRDATA_UART_HXX
