/*! \file	misc.c
 *	\brief	Miscellaneous functions source file
 *
 *	\details
 *
 *	\author University of Minnesota
 *	\author Aerospace Engineering and Mechanics
 *	\copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: misc.c 752 2011-12-21 20:14:23Z murch $
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <math.h>
//#include <sched.h>
//#include <cyg/posix/signal.h>
#include <time.h>

#include "misc.h"

#define STATUS_MSG_SIZE  103
char statusMsg[STATUS_MSG_SIZE]={'\0',};		// TX buffer for status/diagnostic messages to dnlink (system message packet)

double get_time_interval(short id)
{
	struct timespec		ts;
        static struct timespec  ts_p[5];
        double 			nsec,elapsed;

        clock_gettime(CLOCK_REALTIME, &ts);
        nsec    = ts.tv_nsec- ts_p[id].tv_nsec;
        elapsed = nsec*1.0e-9;

        if(elapsed <=0) elapsed =0.0;
        ts_p[id]= ts;

        return elapsed;
}

double get_Time()
{
  	struct timespec t;
    double tnow;

	clock_gettime(CLOCK_REALTIME, &t);
        tnow = t.tv_sec + 1.0e-9*(double)t.tv_nsec;
        return tnow;
}


void reset_Time()
{
	struct timespec tset;

	tset.tv_sec = 0;
	tset.tv_nsec = 0;
	clock_settime(CLOCK_REALTIME,&tset);
}

// swap big/little endian bytes
void endian_swap( unsigned char *buf, int index, int count )
{
	int i;
	unsigned char tmp;

	for ( i = 0; i < count / 2; ++i ) {
		tmp = buf[index+i];
		buf[index+i] = buf[index+count-i-1];
		buf[index+count-i-1] = tmp;
	}
}

// Saturation function
double saturation(double data, double min, double max) {

	if (data > max)
		data = max;
	else if (data < min)
		data = min;

	return data;
}

// polynomial evaulation function, similar to MATLAB's polyval, but must also input the order
// coeff is coefficients in descending powers of the polynomial to be evaluated.
double polyval(double coeff[], double x, int order) {
	double y=0;
	int i;
	
	for(i=order;i>=0;i--){
		if(coeff[i]==0)
			continue;
		y += coeff[order-i] * pow(x,i);
	}

	return y;

}


// Send status message
void send_status(char *status_message){

	char tmp[STATUS_MSG_SIZE]={'U','U','M','\0',};
	
	// Put UUM header in front of the incoming message
	strncat(tmp , status_message , STATUS_MSG_SIZE-strlen(status_message)-1);

	// add new message to buffer, overwrites any existing content
	strcpy(statusMsg,tmp) ;

}

uint16_t do_chksum(unsigned char* buffer, int start, int stop)
{
	unsigned int i;
	uint16_t sum=0;

	for(i=start;i<stop;i++)
		sum += buffer[i];

	return  sum;
}
