/**
 *  \file: sf_6DOFv4.h
 *
 * Sparkfun 6DOF v4 driver
 *
 * Copyright Curt Olson curtolson@gmail.com
 *
 * $Id: gpsd.cpp,v 1.7 2009/08/25 15:04:01 curt Exp $
 */

#include <errno.h>		// errno
#include <fcntl.h>		// open()
#include <stdio.h>		// printf() et. al.
#include <termios.h>		// tcgetattr() et. al.
#include <unistd.h>		// tcgetattr() et. al.
#include <string.h>		// memset(), strerror()

#include "globaldefs.h"

#include "comms/logging.h"
#include "util/strutils.hxx"
#include "util/timing.h"

#include "imu_sf6DOFv4.h"


// imu property nodes
static SGPropertyNode *configroot = NULL;

static SGPropertyNode *imu_device_name_node = NULL;

static SGPropertyNode *imu_timestamp_node = NULL;
static SGPropertyNode *imu_p_node = NULL;
static SGPropertyNode *imu_q_node = NULL;
static SGPropertyNode *imu_r_node = NULL;
static SGPropertyNode *imu_ax_node = NULL;
static SGPropertyNode *imu_ay_node = NULL;
static SGPropertyNode *imu_az_node = NULL;
static SGPropertyNode *imu_hx_node = NULL;
static SGPropertyNode *imu_hy_node = NULL;
static SGPropertyNode *imu_hz_node = NULL;

static int fd = -1;
static string device_name = "/dev/ttyS0";

static double p_bias = 0.0;
static double q_bias = 0.0;
static double r_bias = 0.0;
static double ax_bias = 0.0;
static double ay_bias = 0.0;
static double az_bias = 0.0;
static double hx_bias = 0.0;
static double hy_bias = 0.0;
static double hz_bias = 0.0;

static double p_scale = 1.0;
static double q_scale = 1.0;
static double r_scale = 1.0;
static double ax_scale = 1.0;
static double ay_scale = 1.0;
static double az_scale = 1.0;
static double hx_scale = 1.0;
static double hy_scale = 1.0;
static double hz_scale = 1.0;

// initialize gpsd input property nodes
static void bind_imu_input( SGPropertyNode *config ) {
    imu_device_name_node = config->getChild("device");
    if ( imu_device_name_node != NULL ) {
	device_name = imu_device_name_node->getStringValue();
    }
    configroot = config;

    SGPropertyNode *node = NULL;

    node = config->getChild("p-bias");
    if ( node != NULL ) {
	p_bias = node->getDoubleValue();
    }
    node = config->getChild("q-bias");
    if ( node != NULL ) {
	q_bias = node->getDoubleValue();
    }
    node = config->getChild("r-bias");
    if ( node != NULL ) {
	r_bias = node->getDoubleValue();
    }

    node = config->getChild("ax-bias");
    if ( node != NULL ) {
	ax_bias = node->getDoubleValue();
    }
    node = config->getChild("ay-bias");
    if ( node != NULL ) {
	ay_bias = node->getDoubleValue();
    }
    node = config->getChild("az-bias");
    if ( node != NULL ) {
	az_bias = node->getDoubleValue();
    }

    node = config->getChild("hx-bias");
    if ( node != NULL ) {
	hx_bias = node->getDoubleValue();
    }
    node = config->getChild("hy-bias");
    if ( node != NULL ) {
	hy_bias = node->getDoubleValue();
    }
    node = config->getChild("hz-bias");
    if ( node != NULL ) {
	hz_bias = node->getDoubleValue();
    }

    node = config->getChild("p-scale");
    if ( node != NULL ) {
	p_scale = node->getDoubleValue();
    }
    node = config->getChild("q-scale");
    if ( node != NULL ) {
	q_scale = node->getDoubleValue();
    }
    node = config->getChild("r-scale");
    if ( node != NULL ) {
	r_scale = node->getDoubleValue();
    }

    node = config->getChild("ax-scale");
    if ( node != NULL ) {
	ax_scale = node->getDoubleValue();
    }
    node = config->getChild("ay-scale");
    if ( node != NULL ) {
	ay_scale = node->getDoubleValue();
    }
    node = config->getChild("az-scale");
    if ( node != NULL ) {
	az_scale = node->getDoubleValue();
    }

    node = config->getChild("hx-scale");
    if ( node != NULL ) {
	hx_scale = node->getDoubleValue();
    }
    node = config->getChild("hy-scale");
    if ( node != NULL ) {
	hy_scale = node->getDoubleValue();
    }
    node = config->getChild("hz-scale");
    if ( node != NULL ) {
	hz_scale = node->getDoubleValue();
    }

}


// initialize imu output property nodes 
static void bind_imu_output( string rootname ) {
    SGPropertyNode *outputroot = fgGetNode( rootname.c_str(), true );

    imu_timestamp_node = outputroot->getChild("time-stamp", 0, true);
    imu_p_node = outputroot->getChild("p-rad_sec", 0, true);
    imu_q_node = outputroot->getChild("q-rad_sec", 0, true);
    imu_r_node = outputroot->getChild("r-rad_sec", 0, true);
    imu_ax_node = outputroot->getChild("ax-mps_sec", 0, true);
    imu_ay_node = outputroot->getChild("ay-mps_sec", 0, true);
    imu_az_node = outputroot->getChild("az-mps_sec", 0, true);
    imu_hx_node = outputroot->getChild("hx", 0, true);
    imu_hy_node = outputroot->getChild("hy", 0, true);
    imu_hz_node = outputroot->getChild("hz", 0, true);
}


// send our configured init strings to configure gpsd the way we prefer
static bool sf_6DOFv4_open() {
    if ( display_on ) {
	printf("Sparkfun 6DOFv4 on %s\n", device_name.c_str());
    }

    fd = open( device_name.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK );
    if ( fd < 0 ) {
        fprintf( stderr, "open serial: unable to open %s - %s\n",
                 device_name.c_str(), strerror(errno) );
	return false;
    }

    struct termios oldTio;	// Old Serial Port Settings
    struct termios newTio; 	// New Serial Port Settings
    memset(&oldTio, 0, sizeof(oldTio));
    memset(&newTio, 0, sizeof(newTio));

    // Save Current Serial Port Settings
    tcgetattr(fd,&oldTio); 

    // Configure New Serial Port Settings
    newTio.c_cflag     = B115200 | // bps rate
                         CS8	 | // 8n1
                         CLOCAL	 | // local connection, no modem
                         CREAD;	   // enable receiving chars
    newTio.c_iflag     = IGNPAR;   // ignore parity bits
    newTio.c_oflag     = 0;
    newTio.c_lflag     = 0;
    newTio.c_cc[VTIME] = 0;
    newTio.c_cc[VMIN]  = 0;	   // block 'read' from returning until at
                                   // least 1 character is received

    // Flush Serial Port I/O buffer
    tcflush(fd, TCIOFLUSH);

    // Set New Serial Port Settings
    int ret = tcsetattr( fd, TCSANOW, &newTio );
    if ( ret > 0 ) {
        fprintf( stderr, "error configuring device: %s - %s\n",
                 device_name.c_str(), strerror(errno) );
	return false;
    }

    return true;
}


void sf_6DOFv4_imu_init( string rootname, SGPropertyNode *config ) {
    bind_imu_input( config );
    bind_imu_output( rootname );
    sf_6DOFv4_open();
}


// swap big/little endian bytes
static void my_swap( uint8_t *buf, int index, int count ) {
    int i;
    uint8_t tmp;
    for ( i = 0; i < count / 2; ++i ) {
        tmp = buf[index+i];
        buf[index+i] = buf[index+count-i-1];
        buf[index+count-i-1] = tmp;
    }
}


static bool parse_6DOFv4_msg( uint8_t *msg, int size )
{
    vector<string> tokens = split( (char *)msg );
    if ( tokens.size() != 10 ) {
	if ( display_on ) {
	    printf("Received %d tokens expecting 10. 6DOF output misconfigured?\n", tokens.size());
	    printf("%s\n", msg);
	}
	return false;
    }

    imu_timestamp_node->setDoubleValue( get_Time() );

    double hx_value = ( atof(tokens[1].c_str()) - hx_bias ) * hx_scale;
    imu_hx_node->setDoubleValue( hx_value );
    double hy_value = ( atof(tokens[2].c_str()) - hy_bias ) * hy_scale;
    imu_hy_node->setDoubleValue( hy_value );
    double hz_value = ( atof(tokens[3].c_str()) - hz_bias ) * hz_scale;
    imu_hz_node->setDoubleValue( hz_value );

    double ax_value = ( atof(tokens[4].c_str()) - ax_bias ) * ax_scale;
    imu_ax_node->setDoubleValue( ax_value );
    double ay_value = ( atof(tokens[5].c_str()) - ay_bias ) * ay_scale;
    imu_ay_node->setDoubleValue( ay_value );
    double az_value = ( atof(tokens[6].c_str()) - az_bias ) * az_scale;
    imu_az_node->setDoubleValue( az_value );

    double p_value = ( atof(tokens[7].c_str()) - p_bias ) * p_scale;
    imu_p_node->setDoubleValue( p_value );
    double q_value = ( atof(tokens[8].c_str()) - q_bias ) * q_scale;
    imu_q_node->setDoubleValue( q_value );
    double r_value = ( atof(tokens[9].c_str()) - r_bias ) * r_scale;
    imu_r_node->setDoubleValue( r_value );

    return true;
}


static bool read_6DOFv4() {
    static int state = 0;
    static int counter = 0;
    int len;
    const int max_len = 256;
    uint8_t input[max_len];
    static uint8_t msg[max_len];

    // printf("read 6DOFv4, entry state = %d\n", state);

    bool fresh_data = false;

    if ( state == 0 ) {
	counter = 0;
	len = read( fd, input, 1 );
	while ( len > 0 && input[0] != 'A' ) {
	    // printf( "%c", input[0] );
	    len = read( fd, input, 1 );
	}
	if ( len > 0 && input[0] == 'A' ) {
	    // fprintf( stderr, "read 'A'\n");
	    state = 1;
	}
    }
    if ( state == 1 ) {
	len = read( fd, input, 1 );
	while ( len > 0 && input[0] != 'Z' && counter < max_len-1 ) {
	    msg[counter] = input[0];
	    counter++;
	    // printf( "%c", input[0] );
	    len = read( fd, input, 1 );
	}
	if ( (len > 0 && input[0] == 'Z') || counter >= max_len-1 ) {
	    if ( counter > max_len-1 ) {
		counter = max_len - 1;
	    }
	    msg[counter] = 0;
	    fresh_data = parse_6DOFv4_msg( msg, counter );
	    state = 0;
	}
    }

    return fresh_data;
}


bool sf_6DOFv4_get_imu() {
    // scan for new messages
    bool imu_data_valid = false;

    while ( read_6DOFv4() ) {
	imu_data_valid = true;
    }

    return imu_data_valid;
 }


void sf_6DOFv4_close() {
    close(fd);
}
