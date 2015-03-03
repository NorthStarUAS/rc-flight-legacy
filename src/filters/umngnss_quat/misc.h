/*! \file	misc.h
 *	\brief	Miscellaneous functions header file
 *
 *	\details
 *
 *	\author University of Minnesota
 *	\author Aerospace Engineering and Mechanics
 *	\copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: misc.h 752 2011-12-21 20:14:23Z murch $
 */

#ifndef MISC_H_
#define MISC_H_

#include <stdint.h>

double get_time_interval(short id);
double get_Time();
void reset_Time();
void endian_swap( unsigned char *buf, int index, int count );
double saturation(double data, double min, double max);
double polyval(double coeff[], double x, int order);
void send_status(char *status_message);
uint16_t do_chksum(unsigned char* buffer, int start, int stop);


#endif
