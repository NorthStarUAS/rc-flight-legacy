/**
 *  \file: airdata_uart.cxx
 *
 * Driver for the Bolder Flight Systems airdata module (build on AMSYS pressure
 * sensors.)
 *
 * Copyright (C) 2016 - Curtis L. Olson - curtolson@flightgear.org
 *
 */

#include "python/pyprops.hxx"

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

#include "raven.hxx"


// property nodes
static pyPropertyNode airdata_node;
static pyPropertyNode act_node;

static int fd = -1;
static string device_name = "/dev/ttyO4";

static bool master_opened = false;

#define START_OF_MSG0 147
#define START_OF_MSG1 224

#define FLIGHT_COMMAND_PACKET_ID 23

#define NUM_ACTUATORS 10	// must match the raven firmware
#define PWM_CENTER 1520
#define PWM_HALF_RANGE 413
#define PWM_RANGE (PWM_HALF_RANGE * 2)
#define PWM_MIN (PWM_CENTER - PWM_HALF_RANGE)
#define PWM_MAX (PWM_CENTER + PWM_HALF_RANGE)



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


// initialize actuator property nodes 
static void bind_act_nodes( string output_path ) {
    act_node = pyGetNode(output_path, true);
    act_node.setLen("channel", NUM_ACTUATORS, 0.0);
}

// open the uart
static bool raven_open() {
    if ( master_opened ) {
	return true;
    }
    
    if ( display_on ) {
	printf("raven on %s @ 115,200\n", device_name.c_str());
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

    master_opened = true;
    
    return true;
}


void raven_init( string output_path, pyPropertyNode *config ) {
    bind_airdata_input( config );
    bind_airdata_output( output_path );

    raven_open();
}


static bool raven_parse(uint8_t *buf) {
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

    double pitot_calibrate = 1.0; // make configurable in the future?
    diff_pa -= diff_offset;
    if ( diff_pa < 0.0 ) { diff_pa = 0.0; } // avoid sqrt(neg_number) situation
    float airspeed_mps = sqrt( 2*diff_pa / 1.225 ) * pitot_calibrate;
    float airspeed_kt = airspeed_mps * SG_MPS_TO_KT;
    airdata_node.setDouble( "airspeed_mps", airspeed_mps );
    airdata_node.setDouble( "airspeed_kt", airspeed_kt );

    return true;
}


static bool raven_read() {
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
		new_data = raven_parse( payload );
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


bool raven_update() {
    // scan for new messages
    bool data_valid = false;

    while ( raven_read() ) {
	data_valid = true;
    }

    return data_valid;
 }


void raven_close() {
    close(fd);
}


// generate a pwm pulse length from a normalized [-1 to 1] or [0 to 1] range
static int gen_pulse( double val, bool symmetrical ) {
    int pulse = 0;

    if ( symmetrical ) {
	// i.e. aileron, rudder, elevator
	if ( val < -1.5 ) { val = -1.5; }
	if ( val > 1.5 ) { val = 1.5; }
	pulse = PWM_CENTER + (int)(PWM_HALF_RANGE * val);
    } else {
	// i.e. throttle, flaps
	if ( val < 0.0 ) { val = 0.0; }
	if ( val > 1.0 ) { val = 1.0; }
	pulse = PWM_MIN + (int)(PWM_RANGE * val);
    }

    return pulse;
}


static void raven_cksum( uint8_t hdr1, uint8_t hdr2, uint8_t *buf, uint8_t size, uint8_t *cksum0, uint8_t *cksum1 )
{
    uint8_t c0 = 0;
    uint8_t c1 = 0;

    c0 += hdr1;
    c1 += c0;

    c0 += hdr2;
    c1 += c0;

    for ( uint8_t i = 0; i < size; i++ ) {
        c0 += (uint8_t)buf[i];
        c1 += c0;
    }

    *cksum0 = c0;
    *cksum1 = c1;
}


static bool raven_act_write() {
    uint8_t buf[256];
    uint8_t cksum0, cksum1;
    uint8_t size = 0;
    /* int len; */

    // start of message sync bytes
    buf[0] = START_OF_MSG0; buf[1] = START_OF_MSG1, buf[2] = 0;
    /* len = */ write( fd, buf, 2 );

    // packet id (1 byte)
    buf[0] = FLIGHT_COMMAND_PACKET_ID;
    // packet length (1 byte)
    buf[1] = 2 * NUM_ACTUATORS;
    /* len = */ write( fd, buf, 2 );

    // actuator data
    if ( NUM_ACTUATORS == 8 ) {
	int val;
	uint8_t hi, lo;

	val = gen_pulse( act_node.getDouble("channel", 0), true );
	hi = val / 256;
	lo = val - (hi * 256);
	buf[size++] = lo;
	buf[size++] = hi;

	val = gen_pulse( act_node.getDouble("channel", 1), true );
	hi = val / 256;
	lo = val - (hi * 256);
	buf[size++] = lo;
	buf[size++] = hi;

	val = gen_pulse( act_node.getDouble("channel", 2), false );
	hi = val / 256;
	lo = val - (hi * 256);
	buf[size++] = lo;
	buf[size++] = hi;

	val = gen_pulse( act_node.getDouble("channel", 3), true );
	hi = val / 256;
	lo = val - (hi * 256);
	buf[size++] = lo;
	buf[size++] = hi;

	val = gen_pulse( act_node.getDouble("channel", 4), true );
	hi = val / 256;
	lo = val - (hi * 256);
	buf[size++] = lo;
	buf[size++] = hi;

	val = gen_pulse( act_node.getDouble("channel", 5), false );
	hi = val / 256;
	lo = val - (hi * 256);
	buf[size++] = lo;
	buf[size++] = hi;

	val = gen_pulse( act_node.getDouble("channel", 6), true );
	hi = val / 256;
	lo = val - (hi * 256);
	buf[size++] = lo;
	buf[size++] = hi;

	val = gen_pulse( act_node.getDouble("channel", 7), true );
	hi = val / 256;
	lo = val - (hi * 256);
	buf[size++] = lo;
	buf[size++] = hi;
    }

    // write packet
    /* len = */ write( fd, buf, size );
  
    // check sum (2 bytes)
    raven_cksum( FLIGHT_COMMAND_PACKET_ID, size, buf, size, &cksum0, &cksum1 );
    buf[0] = cksum0; buf[1] = cksum1; buf[2] = 0;
    /* len = */ write( fd, buf, 2 );

    return true;
}


bool raven_act_init( string output_path, pyPropertyNode *section ) {
    if ( ! raven_open() ) {
	return false;
    }

    bind_act_nodes( output_path );

    return true;
}


bool raven_act_update() {
    // send actuator commands to APM2 servo subsystem
    raven_act_write();

    return true;
}

void raven_act_close() {
    // no-op
}
