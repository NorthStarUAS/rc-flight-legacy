/**
 * \file: gps_ublox8.cpp
 *
 * u-blox 7 protocol driver
 *
 * Copyright (C) 2012 - Curtis L. Olson - curtolson@flightgear.org
 *
 */

#include <pyprops.h>

#include <errno.h>		// errno
#include <math.h>		// sin() cos()
#include <sys/types.h>		// open()
#include <sys/stat.h>		// open()
#include <fcntl.h>		// open()
#include <stdio.h>		// printf() et. al.
#include <termios.h>		// tcgetattr() et. al.
#include <unistd.h>		// tcgetattr() et. al.
#include <string.h>		// memset()
#include <sys/time.h>		// gettimeofday()
#include <time.h>
#include <string>

using std::string;

#include "include/globaldefs.h"

#include "comms/display.h"
#include "comms/logging.h"
#include "init/globals.h"
#include "util/strutils.h"
#include "util/timing.h"
#include "gps_mgr.h"

#include "gps_ublox8.h"


// property nodes
static pyPropertyNode gps_node;

static int fd = -1;
static string device_name = "/dev/ttyS0";
static int baud = 115200;
static int gps_fix_value = 0;

// initialize gpsd input property nodes
static void bind_input( pyPropertyNode *config ) {
    if ( config->hasChild("device") ) {
	device_name = config->getString("device");
    }
    if ( config->hasChild("baud") ) {
	baud = config->getLong("baud");
    }
}


// initialize gpsd output property nodes 
static void bind_output( string output_node ) {
    gps_node = pyGetNode(output_node, true);
}


// send our configured init strings to configure gpsd the way we prefer
static bool gps_ublox8_open() {
    if ( display_on ) {
	printf("ublox8 on %s\n", device_name.c_str());
    }

    fd = open( device_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK );
    if ( fd < 0 ) {
        fprintf( stderr, "open serial: unable to open %s - %s\n",
                 device_name.c_str(), strerror(errno) );
	return false;
    }

    struct termios config; 	// Serial port settings
    memset(&config, 0, sizeof(config));

    // Save Current Serial Port Settings
    // tcgetattr(fd,&oldTio); 

    int config_baud = B115200;
    if ( baud == 115200 ) {
	config_baud = B115200;
    } else if ( baud == 57600 ) {
	config_baud = B57600;
    } else if ( baud == 9600 ) {
	config_baud = B9600;
    } else {
	fprintf( stderr, "ublox8 baud rate (%d) unsupported by driver, using back to 115200.\n", baud);
    }

    // Configure New Serial Port Settings
    config.c_cflag     = config_baud | // bps rate
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


void gps_ublox8_init( string output_node, pyPropertyNode *config ) {
    bind_input( config );
    bind_output( output_node );
    gps_ublox8_open();
}


// swap big/little endian bytes
static void my_swap( uint8_t *buf, int index, int count ) {
#if defined( __powerpc__ )
    int i;
    uint8_t tmp;
    for ( i = 0; i < count / 2; ++i ) {
        tmp = buf[index+i];
        buf[index+i] = buf[index+count-i-1];
        buf[index+count-i-1] = tmp;
    }
#endif
}


static bool parse_ublox8_msg( uint8_t msg_class, uint8_t msg_id,
			      uint16_t payload_length, uint8_t *payload )
{
    bool new_position = false;
    static bool set_system_time = false;

    if ( msg_class == 0x01 && msg_id == 0x02 ) {
	// NAV-POSLLH: Please refer to the ublox6 driver (here or in the
	// code history) for a nav-posllh parser
    } else if ( msg_class == 0x01 && msg_id == 0x06 ) {
	// NAV-SOL: Please refer to the ublox6 driver (here or in the
	// code history) for a nav-sol parser that transforms eced
	// pos/vel to lla pos/ned vel.
    } else if ( msg_class == 0x01 && msg_id == 0x07 ) {
	// NAV-PVT
	my_swap( payload, 0, 4);
	my_swap( payload, 4, 2);
	my_swap( payload, 12, 4);
	my_swap( payload, 16, 4);
	my_swap( payload, 24, 4);
	my_swap( payload, 28, 4);
	my_swap( payload, 32, 4);
	my_swap( payload, 36, 4);
	my_swap( payload, 40, 4);
	my_swap( payload, 44, 4);
	my_swap( payload, 48, 4);
	my_swap( payload, 52, 4);
	my_swap( payload, 56, 4);
	my_swap( payload, 60, 4);
	my_swap( payload, 64, 4);
	my_swap( payload, 68, 4);
	my_swap( payload, 72, 4);
	my_swap( payload, 76, 2);
	my_swap( payload, 78, 2);
	my_swap( payload, 80, 4);

	uint8_t *p = payload;
	uint32_t iTOW = *((uint32_t *)p+0);
	int16_t year = *((uint16_t *)(p+4));
	uint8_t month = p[6];
	uint8_t day = p[7];
	uint8_t hour = p[8];
	uint8_t min = p[9];
	uint8_t sec = p[10];
	uint8_t valid = p[11];
	uint32_t tAcc = *((uint32_t *)(p+12));
	int32_t nano = *((int32_t *)(p+16));
	uint8_t fixType = p[20];
	uint8_t flags = p[21];
	uint8_t numSV = p[23];
	int32_t lon = *((int32_t *)(p+24));
	int32_t lat = *((int32_t *)(p+28));
	int32_t height = *((int32_t *)(p+32));
	int32_t hMSL = *((int32_t *)(p+36));
	uint32_t hAcc = *((uint32_t *)(p+40));
	uint32_t vAcc = *((uint32_t *)(p+44));
	int32_t velN = *((int32_t *)(p+48));
	int32_t velE = *((int32_t *)(p+52));
	int32_t velD = *((int32_t *)(p+56));
	uint32_t gSpeed = *((uint32_t *)(p+60));
	int32_t heading = *((int32_t *)(p+64));
	uint32_t sAcc = *((uint32_t *)(p+68));
	uint32_t headingAcc = *((uint32_t *)(p+72));
	uint16_t pDOP = *((uint16_t *)(p+76));

 	gps_fix_value = fixType;
	if ( gps_fix_value == 0 ) {
	    gps_node.setLong( "status", 0 );
	} else if ( gps_fix_value == 1 || gps_fix_value == 2 ) {
	    gps_node.setLong( "status", 1 );
	} else if ( gps_fix_value == 3 ) {
	    gps_node.setLong( "status", 2 );
	}
	// printf("fix: %d lon: %.8f lat: %.8f\n", fixType, (double)lon, (double)lat);

	if ( fixType == 3 ) {
	    // gps thinks we have a good 3d fix so flag our data good.
 	    new_position = true;
	}

	gps_node.setDouble( "timestamp", get_Time() );

	struct tm gps_time;
	gps_time.tm_sec = sec;
	gps_time.tm_min = min;
	gps_time.tm_hour = hour;
	gps_time.tm_mday = day;
	gps_time.tm_mon = month - 1;
	gps_time.tm_year = year - 1900;
	double unix_sec = (double)mktime( &gps_time ) - timezone;
	unix_sec += nano / 1000000000.0;
	gps_node.setDouble( "unix_time_sec", unix_sec );
	gps_node.setDouble( "time_accuracy_ns", tAcc );
	    
	gps_node.setLong( "satellites", numSV );
	    
	gps_node.setDouble( "latitude_deg", (double)lat / 10000000.0);
	gps_node.setDouble( "longitude_deg", (double)lon / 10000000.0);
	gps_node.setDouble( "altitude_m", (float)hMSL / 1000.0 );
	gps_node.setDouble( "vn_ms", (float)velN / 1000.0 );
	gps_node.setDouble( "ve_ms", (float)velE / 1000.0 );
	gps_node.setDouble( "vd_ms", (float)velD / 1000.0 );
	gps_node.setDouble( "horiz_accuracy_m", hAcc / 1000.0 );
	gps_node.setDouble( "vert_accuracy_m", vAcc / 1000.0 );
	gps_node.setDouble( "groundspeed_ms", gSpeed / 1000.0 );
	gps_node.setDouble( "groundtrack_deg", heading / 100000.0 );
	gps_node.setDouble( "heading_accuracy_deg", headingAcc / 100000.0 );
	gps_node.setDouble( "pdop", pDOP / 100.0 );
	gps_node.setLong( "fixType", fixType);
   } else if ( msg_class == 0x01 && msg_id == 0x12 ) {
	// NAV-VELNED: Please refer to the ublox6 driver (here or in the
	// code history) for a nav-velned parser
    } else if ( msg_class == 0x01 && msg_id == 0x21 ) {
	// NAV-TIMEUTC: Please refer to the ublox6 driver (here or in the
	// code history) for a nav-timeutc parser
    } else if ( msg_class == 0x01 && msg_id == 0x30 ) {
	// NAV-SVINFO (partial parse)
	my_swap( payload, 0, 4);

	uint8_t *p = payload;
	// uint32_t iTOW = *((uint32_t *)(p+0));
	uint8_t numCh = p[4];
	// uint8_t globalFlags = p[5];
	int satUsed = 0;
	for ( int i = 0; i < numCh; i++ ) {
	    // uint8_t satid = p[9 + 12*i];
	    // uint8_t flags = p[10 + 12*i];
	    uint8_t quality = p[11 + 12*i];
	    // printf(" chn=%d satid=%d flags=%d quality=%d\n", i, satid, flags, quality);
	    if ( quality > 3 ) {
		satUsed++;
	    }
	}
 	// gps_satellites_node.setLong( satUsed );
	if ( display_on && 0 ) {
	    if ( gps_fix_value < 3 ) {
		printf("Satellite count = %d/%d\n", satUsed, numCh);
	    }
	}
    } else {
	if ( display_on && 0 ) {
	    if ( gps_fix_value < 3 ) {
		printf("ublox8 msg class = %d  msg id = %d\n",
		       msg_class, msg_id);
	    }
	}
    }

    return new_position;
}

static bool read_ublox8() {
    static int state = 0;
    static int msg_class = 0, msg_id = 0;
    static int length_lo = 0, length_hi = 0, payload_length = 0;
    static int counter = 0;
    static uint8_t cksum_A = 0, cksum_B = 0, cksum_lo = 0, cksum_hi = 0;
    int len;
    uint8_t input[500];
    static uint8_t payload[500];

    // printf("read ublox8, entry state = %d\n", state);

    bool new_position = false;

    if ( state == 0 ) {
	counter = 0;
	cksum_A = cksum_B = 0;
	len = read( fd, input, 1 );
	while ( len > 0 && input[0] != 0xB5 ) {
	    // fprintf( stderr, "state0: len = %d val = %2X\n", len, input[0] );
	    len = read( fd, input, 1 );
	}
	if ( len > 0 && input[0] == 0xB5 ) {
	    // fprintf( stderr, "read 0xB5\n");
	    state++;
	}
    }
    if ( state == 1 ) {
	len = read( fd, input, 1 );
	if ( len > 0 ) {
	    if ( input[0] == 0x62 ) {
		// fprintf( stderr, "read 0x62\n");
		state++;
	    } else if ( input[0] == 0xB5 ) {
		// fprintf( stderr, "read 0xB5\n");
	    } else {
		state = 0;
	    }
	}
    }
    if ( state == 2 ) {
	len = read( fd, input, 1 );
	if ( len > 0 ) {
	    msg_class = input[0];
	    cksum_A += input[0];
	    cksum_B += cksum_A;
	    // fprintf( stderr, "msg class = %d\n", msg_class );
	    state++;
	}
    }
    if ( state == 3 ) {
	len = read( fd, input, 1 );
	if ( len > 0 ) {
	    msg_id = input[0];
	    cksum_A += input[0];
	    cksum_B += cksum_A;
	    // fprintf( stderr, "msg id = %d\n", msg_id );
	    state++;
	}
    }
    if ( state == 4 ) {
	len = read( fd, input, 1 );
	if ( len > 0 ) {
	    length_lo = input[0];
	    cksum_A += input[0];
	    cksum_B += cksum_A;
	    state++;
	}
    }
    if ( state == 5 ) {
	len = read( fd, input, 1 );
	if ( len > 0 ) {
	    length_hi = input[0];
	    cksum_A += input[0];
	    cksum_B += cksum_A;
	    payload_length = length_hi*256 + length_lo;
	    // fprintf( stderr, "payload len = %d\n", payload_length );
	    if ( payload_length > 400 ) {
		state = 0;
	    } else {
		state++;
	    }
	}
    }
    if ( state == 6 ) {
	len = read( fd, input, 1 );
	while ( len > 0 ) {
	    payload[counter++] = input[0];
	    //fprintf( stderr, "%02X ", input[0] );
	    cksum_A += input[0];
	    cksum_B += cksum_A;
	    if ( counter >= payload_length ) {
		break;
	    }
	    len = read( fd, input, 1 );
	}

	if ( counter >= payload_length ) {
	    state++;
	    //fprintf( stderr, "\n" );
	}
    }
    if ( state == 7 ) {
	len = read( fd, input, 1 );
	if ( len > 0 ) {
	    cksum_lo = input[0];
	    state++;
	}
    }
    if ( state == 8 ) {
	len = read( fd, input, 1 );
	if ( len > 0 ) {
	    cksum_hi = input[0];
	    if ( cksum_A == cksum_lo && cksum_B == cksum_hi ) {
		// fprintf( stderr, "checksum passes (%d)!\n", msg_id );
		new_position = parse_ublox8_msg( msg_class, msg_id,
						 payload_length, payload );
		state++;
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

    return new_position;
}


bool gps_ublox8_update() {
    // run an iteration of the ublox scanner/parser
    bool gps_data_valid = read_ublox8();

    return gps_data_valid;
}


void gps_ublox8_close() {
}
