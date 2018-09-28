/**
 *  \file: airdata_uart.cxx
 *
 * Driver for the Bolder Flight Systems airdata module (build on AMSYS pressure
 * sensors.)
 *
 * Copyright (C) 2016 - Curtis L. Olson - curtolson@flightgear.org
 *
 */

#include <pyprops.hxx>

#include <errno.h>		// errno
#include <fcntl.h>		// open()
#include <math.h>		// fabs()
#include <stdio.h>		// printf() et. al.
#include <termios.h>		// tcgetattr() et. al.
#include <unistd.h>		// tcgetattr() et. al.
#include <string.h>		// memset(), strerror()

#include "include/globaldefs.h"

#include "comms/display.hxx"
#include "util/strutils.hxx"
#include "util/timing.h"

#include "airdata_bolder.hxx"


// property nodes
static pyPropertyNode airdata_node;

static int fd = -1;
static string device_name = "/dev/ttyO4";

static bool airspeed_inited = false;
static double airspeed_zero_start_time = 0.0;


// initialize gpsd input property nodes
static void bind_airdata_input( pyPropertyNode *config ) {
    if ( config->hasChild("device") ) {
	device_name = config->getString("device");
    }
}


// initialize imu output property nodes 
static void bind_airdata_output( string output_path ) {
    airdata_node = pyGetNode(output_path, true);
}


// open the uart
static bool airdata_bolder_open() {
    if ( display_on ) {
	printf("airdata on %s @ 115,200\n", device_name.c_str());
    }

    fd = open( device_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK );
    if ( fd < 0 ) {
        fprintf( stderr, "open serial: unable to open %s - %s\n",
                 device_name.c_str(), strerror(errno) );
	return false;
    }

    struct termios config; 	// New Serial Port Settings

    memset(&config, 0, sizeof(config));

    // Save Current Serial Port Settings
    // tcgetattr(fd,&oldTio); 

    // Configure New Serial Port Settings
    config.c_cflag     = B115200 | // bps rate
                         CS8	 | // 8n1
                         CLOCAL	 | // local connection, no modem
                         CREAD;	   // enable receiving chars
    config.c_iflag     = IGNPAR;   // ignore parity bits
    config.c_oflag     = 0;
    config.c_lflag     = 0;
    config.c_cc[VTIME] = 0;
    config.c_cc[VMIN]  = 0;	   // block 'read' from returning until at
                                   // least 1 character is received

    // Flush Serial Port I/O buffer
    tcflush(fd, TCIOFLUSH);

    // Set New Serial Port Settings
    int ret = tcsetattr( fd, TCSANOW, &config );
    if ( ret > 0 ) {
        fprintf( stderr, "error configuring device: %s - %s\n",
                 device_name.c_str(), strerror(errno) );
	return false;
    }

    // Enable non-blocking IO (one more time for good measure)
    fcntl(fd, F_SETFL, O_NONBLOCK);

    return true;
}


void airdata_bolder_init( string output_path, pyPropertyNode *config ) {
    bind_airdata_input( config );
    bind_airdata_output( output_path );

    airdata_bolder_open();
}


static bool airdata_parse(uint8_t *buf) {
    static double diff_sum = 0.0;
    static int diff_count = 0;
    static float diff_offset = 0.0;

    double static_pa = *(float *)buf; buf += 4;
    double diff_pa = *(float *)buf; buf += 4;

    // printf("static(pa) = %.2f\n", static_pa);
    // printf("diff(pa) = %.2f\n", diff_pa);
    
    airdata_node.setDouble( "timestamp", get_Time() );
    airdata_node.setDouble( "pressure_mbar", (static_pa / 100.0) );
    airdata_node.setDouble( "diff_pa", diff_pa);

    if ( ! airspeed_inited ) {
	if ( airspeed_zero_start_time > 0 ) {
	    diff_sum += diff_pa;
	    diff_count++;
	    diff_offset = diff_sum / diff_count;
	} else {
	    airspeed_zero_start_time = get_Time();
	    diff_sum = 0.0;
	    diff_count = 0;
	}
	if ( get_Time() > airspeed_zero_start_time + 10.0 ) {
	    //printf("diff_offset = %.2f\n", diff_offset);
	    airspeed_inited = true;
	}
    }

    double pitot_calibrate = 1.0; // make configurable in the future?
    diff_pa -= diff_offset;
    if ( diff_pa < 0.0 ) { diff_pa = 0.0; } // avoid sqrt(neg_number) situation
    float airspeed_mps = sqrt( 2*diff_pa / 1.225 ) * pitot_calibrate;
    float airspeed_kt = airspeed_mps * SG_MPS_TO_KT;
    airdata_node.setDouble( "airspeed_mps", airspeed_mps );
    airdata_node.setDouble( "airspeed_kt", airspeed_kt );

    return true;
}


// force an airspeed zero calibration (ideally with the aircraft on
// the ground with the pitot tube perpendicular to the prevailing
// wind.)
void airdata_bolder_zero_airspeed() {
    airspeed_inited = false;
    airspeed_zero_start_time = 0.0;
}


static bool airdata_bolder_read() {
    static const int payload_length = 8;

    static int state = 0;
    static int counter = 0;
    static uint8_t cksum_A = 0, cksum_B = 0, cksum_lo = 0, cksum_hi = 0;
    int len;
    uint8_t input[500];
    static uint8_t payload[500];

    // printf("read airdata, entry state = %d\n", state);

    bool new_data = false;

    if ( state == 0 ) {
	counter = 0;
	cksum_A = cksum_B = 0;
	len = read( fd, input, 1 );
	while ( len > 0 && input[0] != 0x42 ) {
	    // printf( "state0: len = %d val = %2X\n", len, input[0] );
	    len = read( fd, input, 1 );
	}
	if ( len > 0 && input[0] == 0x42 ) {
	    // printf( "read 0xB5\n");
	    state++;
	}
    }
    if ( state == 1 ) {
	len = read( fd, input, 1 );
	if ( len > 0 ) {
	    if ( input[0] == 0x46 ) {
		// printf( "read 0x46\n");
		state++;
	    } else if ( input[0] == 0x42 ) {
		// printf( "read 0x42\n");
	    } else {
		state = 0;
	    }
	}
    }
    if ( state == 2 ) {
	len = read( fd, input, 1 );
	while ( len > 0 ) {
	    payload[counter++] = input[0];
	    // printf( "%02X ", input[0] );
	    cksum_A += input[0];
	    cksum_B += cksum_A;
	    if ( counter >= payload_length ) {
		break;
	    }
	    len = read( fd, input, 1 );
	}
	if ( counter >= payload_length ) {
	    state++;
	    // printf( "\n" );
	}
    }
    if ( state == 3 ) {
	len = read( fd, input, 1 );
	if ( len > 0 ) {
	    cksum_lo = input[0];
	    state++;
	}
    }
    if ( state == 4 ) {
	len = read( fd, input, 1 );
	if ( len > 0 ) {
	    cksum_hi = input[0];
	    if ( cksum_A == cksum_lo && cksum_B == cksum_hi ) {
		// printf("checksum passes!\n");
		new_data = airdata_parse( payload );
	    } else {
		if ( display_on && 0 ) {
		    printf("checksum failed %d %d (computed) != %d %d (message)\n",
			   cksum_A, cksum_B, cksum_lo, cksum_hi );
		}
	    }
	    // this is the end of a record, reset state to 0 to start
	    // looking for next record
	    state = 0;
	}
    }

    return new_data;
}


bool airdata_bolder_update() {
    // scan for new messages
    bool airdata_valid = false;

    while ( airdata_bolder_read() ) {
	airdata_valid = true;
    }

    return airdata_valid;
 }


void airdata_bolder_close() {
    close(fd);
}
