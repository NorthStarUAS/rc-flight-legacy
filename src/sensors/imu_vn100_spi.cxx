/**
 *  \file: imu_vn100_spi.cxx
 *
 * Vectornav.com VN-100 (ascii/spi) driver
 *
 * Copyright (C) 2012 - Curtis L. Olson - curtolson@flightgear.org
 *
 */

#include "python/pyprops.hxx"

#include <errno.h>		// errno
#include <fcntl.h>		// open()
#include <math.h>		// fabs()
#include <stdio.h>		// printf() et. al.
#include <string.h>		// memset(), strerror()
#include <unistd.h>		// open() / write()

#include "include/globaldefs.h"

#include "comms/display.h"
#include "util/strutils.hxx"
#include "util/timing.h"

#include "imu_vn100_spi.hxx"


// imu property nodes
static pyPropertyNode imu_node;

static int fd = -1;
static string device_name = "/dev/spike";


// initialize gpsd input property nodes
static void bind_imu_input( pyPropertyNode *config ) {
    if ( config->hasChild("device") ) {
	device_name = config->getString("device");
    }
}


// initialize imu output property nodes 
static void bind_imu_output( pyPropertyNode *base ) {
    imu_node = *base;
}


// open (and start) the spi driver
static bool imu_vn100_spi_open() {
    if ( display_on ) {
	printf("Vectornav.com VN-100 on %s\n", device_name.c_str());
    }

    fd = open( device_name.c_str(), O_RDWR );
    if ( fd < 0 ) {
        fprintf( stderr, "open vn100 spi: unable to open %s - %s\n",
                 device_name.c_str(), strerror(errno) );
	return false;
    }

    string command = "start\n";
    unsigned int result = write( fd, command.c_str(), command.length() );
    if ( result != command.length() ) {
	fprintf( stderr, "spi driver start command failed: %s\n",
		 strerror(errno) );
	return false;
    }

    return true;
}


void imu_vn100_spi_init( pyPropertyNode *base, pyPropertyNode *config ) {
    bind_imu_input( config );
    bind_imu_output( base );

    imu_vn100_spi_open();
}


static bool imu_vn100_spi_parse_msg( uint8_t *msg_buf, int size )
{
    double current_time = get_Time();

    // variables used to compute an intial steady state gyro bias
    static bool bias_ready = false;
    static double start_time  = -1.0;
    static int count = 0;
    static double p_sum = 0.0, q_sum = 0.0, r_sum = 0.0;
    static double p_bias = 0.0, q_bias = 0.0, r_bias = 0.0;

    // validate message
    if ( size != 56 ) {
        // bogus command
        return false;
    }

    if ( msg_buf[12] != 0x00 || msg_buf[13] != 0x01 || msg_buf[14] != 0xfc ) {
	// not a response to our command (i.e. reset/powerup data, etc.)
	return false;
    }

    uint32_t *iptr = NULL;
    iptr = (uint32_t *)&msg_buf[0]; /* running */
    imu_node.setLong( "driver_running", *iptr );

    iptr = (uint32_t *)&msg_buf[4]; /* call back counter */
    imu_node.setLong( "driver_callbacks", *iptr );

    iptr = (uint32_t *)&msg_buf[8]; /* busy counter */
    imu_node.setLong( "driver_overruns", *iptr );
    
    float p, q, r;
    int i = 16;

    float *fptr = NULL;

    fptr = (float *)&msg_buf[i]; i += 4;
    // hx_filter = 0.75*hx_filter + 0.25*(*fptr);
    imu_node.setDouble( "hx", *fptr );

    fptr = (float *)&msg_buf[i]; i += 4;
    // hy_filter = 0.75*hy_filter + 0.25**fptr;
    imu_node.setDouble( "hy", *fptr );

    fptr = (float *)&msg_buf[i]; i += 4;
    // hz_filter = 0.75*hz_filter + 0.25**fptr;
    imu_node.setDouble( "hz", *fptr );

    fptr = (float *)&msg_buf[i]; i += 4;
    // ax_filter = 0.75*ax_filter + 0.25**fptr;
    imu_node.setDouble( "ax_mps_sec", *fptr );

    fptr = (float *)&msg_buf[i]; i += 4;
    // ay_filter = 0.75*ay_filter + 0.25**fptr;
    imu_node.setDouble( "ay_mps_sec", *fptr );

    fptr = (float *)&msg_buf[i]; i += 4;
    // az_filter = 0.75*az_filter + 0.25**fptr;
    imu_node.setDouble( "az_mps_sec", *fptr );

    fptr = (float *)&msg_buf[i]; i += 4;
    p = *fptr;
    // p_filter = 0.75*p_filter + 0.25*p;
    imu_node.setDouble( "p_rad_sec", p - p_bias );

    fptr = (float *)&msg_buf[i]; i += 4;
    q = *fptr;
    // q_filter = 0.75*q_filter + 0.25*q;
    imu_node.setDouble( "q_rad_sec", q - q_bias );

    fptr = (float *)&msg_buf[i]; i += 4;
    r = *fptr;
    // r_filter = 0.75*r_filter + 0.25*r;
    imu_node.setDouble( "r_rad_sec", r - r_bias );

    fptr = (float *)&msg_buf[i]; i += 4;
    imu_node.setDouble( "temp_C", *fptr );

    imu_node.setDouble( "timestamp", current_time );

    if ( !bias_ready ) {
	// average first 15 seconds of steady state gyro values and
	// use as a global bias.  This should be removed for the
	// temperature compensated vector nav unit.
	if ( start_time < 0.0 ) {
	    start_time = current_time;
	}
	if ( current_time - start_time < 15.0 ) {
	    p_sum += p;
	    q_sum += q;
	    r_sum += r;
	    count++;
	    p_bias = p_sum / (double)count;
	    q_bias = q_sum / (double)count;
	    r_bias = r_sum / (double)count;
	    imu_node.setDouble( "p_bias", p_bias );
	    imu_node.setDouble( "q_bias", q_bias );
	    imu_node.setDouble( "r_bias", r_bias );
	} else {
	    bias_ready = true;
	    if ( display_on ) {
		printf("gyro bias: p=%.2f q=%.2f r=%.2f\n",
		       p_bias, q_bias, r_bias);
	    }
	    // sanity check
	    if ( fabs(p_bias) > 1.0 /* 57.3 deg/sec */ ||
		 fabs(q_bias) > 1.0 /* 57.3 deg/sec */ ||
		 fabs(r_bias) > 1.0 /* 57.3 deg/sec */ )
	    {
		printf("Something is wrong with the gyro, it is outputting bad data!\n");
		printf("Aborting so you can fix the hardware problem.\n");
		printf("NOTE: IMU must be still when software is started.\n");
		exit(-1);
	    }
	}
    }

    return true;
}


static bool imu_vn100_spi_read() {
    const unsigned int spi_len = 56;
    uint8_t input[spi_len];

    unsigned int result = read( fd, input, spi_len );
    if ( result != spi_len ) {
	// printf("failed vn100_spi read, result = %d\n", result);
	return false;
    }

    // printf("read vn100_spi, result = %d\n", result);

    imu_vn100_spi_parse_msg( input, spi_len );

    return true;
}


bool imu_vn100_spi_get() {
    // read newest data
    bool imu_data_valid = false;

    if ( imu_vn100_spi_read() ) {
	imu_data_valid = true;
    }

    return imu_data_valid;
 }


void imu_vn100_spi_close() {
    close(fd);
}
